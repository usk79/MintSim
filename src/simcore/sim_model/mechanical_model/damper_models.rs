/// # ばねモデル
/// - 単純なばねモデル（質量の無い　理想ばね）
/// 

use crate::prelude::{*, sim_signal::signal::SigTrait};

use anyhow::{anyhow, Context};

/// 単純なダンパモデル（自然長からの変位（ダンパ端1, 2の距離）に応じて力を出力する単純なもの）
/// ダンパの変位は伸縮する方向のみであると仮定したモデル
#[derive(Debug, Clone)]
pub struct SimpleDamper {
    damping_coeff: f64, // 減衰係数[N/(m/s)]
    damper_length: f64, // ダンパ長[m]
    input_bus: RefBus, // 必ず6要素で使用する( ダンパ端1の座標(x1, y1, z1)、ダンパ端2の座標（x2, y2, z2) )
    output_bus: Bus, // 必ず6要素で使用する( ダンパ端1側の力(Fx1, Fy1, Fz1), ダンパ端2側の力(Fx2, Fy2, Fz2) この力は反対方向を向いている＝合力は0)
}

impl SimpleDamper {
    pub fn new(input_def: Vec<SigDef>, output_def: Vec<SigDef>, damping_coeff: f64) -> anyhow::Result<Self> {
        let inbus = RefBus::try_from(input_def).context(format!("SimpleDamperの入力バスが不正です。"))?;
        let outbus = Bus::try_from(output_def).context(format!("SimpleDamperの出力バスが不正です。"))?;

        if inbus.len() != 6 {
            return Err(anyhow!("SimpleDamper:inputbusの要素数は6である必要があります。\n"));
        }
        if outbus.len() != 6 {
            return Err(anyhow!("SimpleDamper:outbusの要素数は6である必要があります。\n"));
        }

        if damping_coeff < 0.0 {
            return  Err(anyhow!("SimpleDamper:減衰係数>=0である必要があります。\n"));
        }

        Ok(Self{
            damping_coeff: damping_coeff,
            damper_length: 0.0, 
            input_bus: inbus,
            output_bus: outbus,
        })
    }
}

impl ModelCore for SimpleDamper {
    fn initialize(&mut self, _sim_time: &sim_system::SimTime) { 
        let dx = self.input_bus[0].val() - self.input_bus[3].val();
        let dy = self.input_bus[1].val() - self.input_bus[4].val();
        let dz = self.input_bus[2].val() - self.input_bus[5].val();

        self.damper_length = ((dx * dx) + (dy * dy) + (dz * dz)).sqrt();
    }

    fn finalize(&mut self) { }

    fn interface_in(&mut self) -> Option<&mut RefBus> {
        Some(&mut self.input_bus)
    }

    fn interface_out(&self) -> Option<&Bus> {
        Some(&self.output_bus)
    }

    fn nextstate(&mut self, sim_time: &sim_system::SimTime) {
        let dx = self.input_bus[0].val() - self.input_bus[3].val();
        let dy = self.input_bus[1].val() - self.input_bus[4].val();
        let dz = self.input_bus[2].val() - self.input_bus[5].val();

        let distance = ((dx * dx) + (dy * dy) + (dz * dz)).sqrt();
        let displacement = self.damper_length - distance; // ダンパの変位
        let force = self.damping_coeff * displacement / sim_time.delta_t(); // ダンパの発生する力
        let fx = force * dx / distance;
        let fy = force * dy / distance;
        let fz = force * dz / distance;

        self.damper_length = distance;

        self.output_bus[0].set_val(fx);
        self.output_bus[1].set_val(fy);
        self.output_bus[2].set_val(fz);
        self.output_bus[3].set_val(-fx);
        self.output_bus[4].set_val(-fy);
        self.output_bus[5].set_val(-fz);

    }
}

#[cfg(test)]
mod damper_models_test {
    
    use crate::prelude::{*};

    use super::SimpleDamper;

    /// ダンパ（減衰係数1.0N/(m/s))におもり（1kg)を接続　おもりに初速10m/sを与えてスタートするシミュレーション
    /// ダンパの効果により速度が徐々に落ちていく動きとなる
    #[test]
    fn simpledampler_test() {
        let wall = ConstantFunc::new(
            MakeSigList![("wall_x", "m"), ("wall_y", "m"), ("wall_z", "m")],
            &[0.0, 0.0, 0.0]
        ).unwrap();

        let mut ball = MassModel::new(
            MakeSigList![("ball_fx", "N"), ("ball_fy", "N"), ("ball_fz", "N")],
            MakeSigList![("ball_x", "m"), ("ball_y", "m"), ("ball_z", "m"), ("ball_vx", "m/s"), ("ball_vy", "m/s"), ("ball_vz", "m/s")],
            1.0,
            (10.0, 0.0, 0.0),
            (1.0, 0.0, 0.0),
            SolverType::RungeKutta
        ).unwrap();

        let mut spring = SimpleDamper::new(
            MakeSigList![("wall_x", "m"), ("wall_y", "m"), ("wall_z", "m"), ("ball_x", "m"), ("ball_y", "m"), ("ball_z", "m")],
            MakeSigList![("wall_fx", "N"), ("wall_fy", "N"), ("wall_fz", "N"), ("ball_fx", "N"), ("ball_fy", "N"), ("ball_fz", "N")],
            1.0,
        ).unwrap();

        let mut scp = SimRecorder::new(
            MakeSigList![("ball_x", "m"), ("ball_fx", "N")]
        ).unwrap();

        // モデルの接続
        connect_models(&wall, &["wall_x", "wall_y", "wall_z"], &mut spring, &["wall_x", "wall_y", "wall_z"]).unwrap();
        connect_models(&ball, &["ball_x", "ball_y", "ball_z"], &mut spring, &["ball_x", "ball_y", "ball_z"]).unwrap();
        connect_models(&spring, &["ball_fx", "ball_fy", "ball_fz"], &mut ball, &["ball_fx", "ball_fy", "ball_fz"]).unwrap();
        
        connect_models(&ball, &["ball_x"], &mut scp, &["ball_x"]).unwrap();
        connect_models(&spring, &["ball_fx"], &mut scp, &["ball_fx"]).unwrap();

        // 登録
        let mut sys = SimSystem::new(0.0, 10.0, 0.001);

        sys.regist_model(wall);
        sys.regist_model(ball);
        sys.regist_model(spring);
        
        sys.regist_recorder("scp1", scp);

        sys.run();

        sys.get_recorder("scp1").unwrap().timeplot_all(
            "test_output\\simpledamper_mdl.png", 
            (300, 500),
            (2, 1)
        ).unwrap();

        sys.get_recorder("scp1").unwrap().export("test_output\\simpledamper_mdl.csv").unwrap();
    }

}