/// # 質点モデル
/// - 質量と座標を持ったモデル
/// 
/// 剛体モデルもここで実装する？

use crate::prelude::{*};

use anyhow::{anyhow, Context};


#[derive(Debug, Clone)]
pub struct MassModel {
    model: SpaceStateModel,
}

impl MassModel {
    pub fn new(input_def: Vec<SigDef>, output_def: Vec<SigDef>, mass:f64, init_pos: (f64, f64, f64), init_velocity: (f64, f64, f64), solvertype: SolverType) -> anyhow::Result<Self> {
        
        if input_def.len() != 3 {
            return Err(anyhow!("MassModel: 入力信号の要素数は3個（x, y, z)方向の力で設定してください"))
        }

        if output_def.len() != 6 {
            return Err(anyhow!("MassModel: 出力信号の要素数は6個(x, y, z, vx, vy, vz)で設定してください"))
        }

        let mut model = SpaceStateModel::new(input_def, output_def, 6, solvertype).context("MassModel:エラーが発生しました。")?;

        model.set_mtrx_a(&[ 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
                            0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
                            0.0, 0.0, 0.0, 0.0, 0.0, 1.0,
                            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, ]).context("MassModel:エラーが発生しました。")?;
        model.set_mtrx_b(&[ 0.0,        0.0,        0.0,
                            0.0,        0.0,        0.0,
                            0.0,        0.0,        0.0,
                            1.0 / mass, 0.0,        0.0,
                            0.0,        1.0 / mass, 0.0,
                            0.0,        0.0,        1.0 / mass]).context("MassModel:エラーが発生しました。")?;
        let cmat = (0..36).into_iter().map(|v| if v % (6 + 1) == 0 {1.0} else {0.0} ).collect::<Vec<f64>>(); // 単位行列の作成（6は次元数）
        model.set_mtrx_c(&cmat).context("MassModel:エラーが発生しました。")?;

        let vec = vec![init_pos.0, init_pos.1, init_pos.2, init_velocity.0, init_velocity.1, init_velocity.2];
        model.set_init_state(&vec).context("MassModel:エラーが発生しました。")?;

        Ok(Self {
            model: model,
        })
    }


}

impl ModelCore for MassModel {
    fn initialize(&mut self, sim_time: &sim_system::SimTime) {
        self.model.initialize(sim_time);
    }

    fn finalize(&mut self) {
        self.model.finalize();
    }

    fn interface_in(&mut self) -> Option<&mut RefBus> {
        self.model.interface_in()
    }

    fn interface_out(&self) -> Option<&Bus> {
        self.model.interface_out()
    }

    fn nextstate(&mut self, sim_time: &sim_system::SimTime) {
        self.model.nextstate(sim_time);
    }
}

#[cfg(test)]
mod mass_models_test {
    use super::MassModel;
    use crate::prelude::{*};

    #[test]
    fn make_test() {
        let input = StepFunc::new(
            vec![SigDef::new("Fx", "N"), SigDef::new("Fy", "N"), SigDef::new("Fz", "N")],
            vec![(0.0, 0.1, 1.0), (0.0, 0.2, 2.0), (0.1, -0.1, 3.0)]
        ).unwrap();

        let mut model = MassModel::new(
            vec![SigDef::new("Fx", "N"), SigDef::new("Fy", "N"), SigDef::new("Fz", "N")],
            vec![SigDef::new("x", "m"), SigDef::new("y", "m"), SigDef::new("z", "m"), SigDef::new("Vx", "m/s"), SigDef::new("Vy", "m/s"), SigDef::new("Vz", "m/s")],
            1.0,
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 0.0),
            SolverType::Euler
        ).unwrap();
        // スコープ
        let mut scp = SimRecorder::new(
            vec![
                SigDef::new("Fx", "N"), SigDef::new("Fy", "N"), SigDef::new("Fz", "N"),
                SigDef::new("x", "m"), SigDef::new("y", "m"), SigDef::new("z", "m"),
                SigDef::new("Vx", "m/s"), SigDef::new("Vy", "m/s"), SigDef::new("Vz", "m/s"),
            ]
        ).unwrap();

        // 接続
        connect_models(&input, &["Fx", "Fy", "Fz"], &mut model, &["Fx", "Fy", "Fz"]).unwrap();
        connect_models(&model, &["x", "y", "z", "Vx", "Vy", "Vz"], &mut scp, &["x", "y", "z", "Vx", "Vy", "Vz"]).unwrap();
        connect_models(&input, &["Fx", "Fy", "Fz"], &mut scp, &["Fx", "Fy", "Fz"]).unwrap();
        // 登録
        let mut sys = SimSystem::new(0.0, 10.0, 0.001);

        sys.regist_model(input);
        sys.regist_model(model);
        sys.regist_recorder("scp1", scp);

        sys.run();

        sys.get_recorder("scp1").unwrap().timeplot_all(
            "test_output\\massmdl.png", 
            (1000, 800),
            (3, 3)
        ).unwrap();

        sys.get_recorder("scp1").unwrap().export("test_output\\massmdl.csv").unwrap();

    }
}