/// # ばねダンパモデル
/// - 単純なばねダンパモデル（質量の無い　理想ばね）
/// 

use crate::prelude::{*, sim_signal::signal::SigTrait};

use anyhow::{anyhow, Context};

use super::{spring_models, damper_models};

/// 単純なばねモデル（自然長からの変位（ばね端1, 2の距離）に応じて力を出力する単純なもの）
/// と単純なダンパモデルを組み合わせたモデル
#[derive(Debug, Clone)]
pub struct SimpleSpringDamper {
    natural_length: f64, // ばねの自然長[m]（length >= 0 値のみ許可）
    spring_constant: f64, // ばね定数[N/m]
    damping_coeff: f64, // 減衰係数[N/(m/s)]
    damper_length: f64, // ダンパ長[m]
    input_bus: RefBus, // 必ず6要素で使用する( ばね端1の座標(x1, y1, z1)、ばね端2の座標（x2, y2, z2) )
    output_bus: Bus, // 必ず6要素で使用する( ばね端1側の力(Fx1, Fy1, Fz1), ばね端2側の力(Fx2, Fy2, Fz2) この力は反対方向を向いている＝合力は0)
}

impl SimpleSpringDamper {
    pub fn new(input_def: Vec<SigDef>, output_def: Vec<SigDef>, natural_length: f64, spring_constant: f64, damping_coeff: f64) -> anyhow::Result<Self> {
        let inbus = RefBus::try_from(input_def).context(format!("SimpleSpringDamperの入力バスが不正です。"))?;
        let outbus = Bus::try_from(output_def).context(format!("SimpleSpringDamperの出力バスが不正です。"))?;

        if inbus.len() != 6 {
            return Err(anyhow!("SimpleSpringDamper:inputbusの要素数は6である必要があります。\n"));
        }
        if outbus.len() != 6 {
            return Err(anyhow!("SimpleSpringDamper:outbusの要素数は6である必要があります。\n"));
        }

        if natural_length < 0.0 {
            return  Err(anyhow!("SimpleSpringDamper:ばねの自然長>=0である必要があります。\n"));
        }

        if spring_constant < 0.0 {
            return  Err(anyhow!("SimpleSpringDamper:ばね定数>=0である必要があります。\n"));
        }

        Ok(Self{
            natural_length: natural_length,
            spring_constant: spring_constant,
            damping_coeff: damping_coeff,
            damper_length: 0.0,
            input_bus: inbus,
            output_bus: outbus,
        })
    }
}

impl ModelCore for SimpleSpringDamper {
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
        let displacement = self.natural_length - distance; // ばねの変位
        let spring_force = self.spring_constant * displacement; // ばねの発生する力

        let displacement = self.damper_length - distance; // ダンパの変位
        let damper_force = self.damping_coeff * displacement / sim_time.delta_t(); // ダンパの発生する力

        let force = spring_force + damper_force;

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

    use super::SimpleSpringDamper;

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
            (1.5, 0.0, 0.0),
            (1.0, 0.0, 0.0),
            SolverType::RungeKutta
        ).unwrap();

        let mut mdl = SimpleSpringDamper::new(
            MakeSigList![("wall_x", "m"), ("wall_y", "m"), ("wall_z", "m"), ("ball_x", "m"), ("ball_y", "m"), ("ball_z", "m")],
            MakeSigList![("wall_fx", "N"), ("wall_fy", "N"), ("wall_fz", "N"), ("ball_fx", "N"), ("ball_fy", "N"), ("ball_fz", "N")],
            0.8,
            1.0,
            0.4,
        ).unwrap();

        let mut scp = SimRecorder::new(
            MakeSigList![("ball_x", "m"), ("ball_fx", "N")]
        ).unwrap();

        // モデルの接続
        connect_models(&wall, &["wall_x", "wall_y", "wall_z"], &mut mdl, &["wall_x", "wall_y", "wall_z"]).unwrap();
        connect_models(&ball, &["ball_x", "ball_y", "ball_z"], &mut mdl, &["ball_x", "ball_y", "ball_z"]).unwrap();
        connect_models(&mdl, &["ball_fx", "ball_fy", "ball_fz"], &mut ball, &["ball_fx", "ball_fy", "ball_fz"]).unwrap();
        
        connect_models(&ball, &["ball_x"], &mut scp, &["ball_x"]).unwrap();
        connect_models(&mdl, &["ball_fx"], &mut scp, &["ball_fx"]).unwrap();

        // 登録
        let mut sys = SimSystem::new(0.0, 30.0, 0.001);

        sys.regist_model(wall);
        sys.regist_model(ball);
        sys.regist_model(mdl);
        
        sys.regist_recorder("scp1", scp);

        sys.run();

        sys.get_recorder("scp1").unwrap().timeplot_all(
            "test_output\\simplespringdamper_mdl.png", 
            (300, 500),
            (2, 1)
        ).unwrap();

        sys.get_recorder("scp1").unwrap().export("test_output\\simplespringdamper_mdl.csv").unwrap();
    }

}