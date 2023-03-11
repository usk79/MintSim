/// # Sampleモデル
/// Sampleモデルには、下記のモデルを実装する
/// 
/// - RLC回路
/// - ボールアンドビーム
use super::model_core::{ModelCore};
use super::de_models::{SpaceStateModel, SolverType, DEModel};

use super::super::sim_signal;
use sim_signal::signal::{SigDef, SigTrait};

use sim_signal::bus::{Bus, RefBus};

use super::super::sim_system;
use sim_system::SimTime;

use super::super::sim_common::{G, UnitTrans};


/// RLC直列回路の状態空間モデルを生成する関数（サンプル）
/// https://qiita.com/code0327/items/423b0f0380e8c64f3580 数値はここを参考
/// 第1引数：抵抗[Ω]
/// 第2引数：インダクタンス[H]
/// 第3引数：キャパシタンス[C]
/// 第4引数：初期電流[A]
/// 第5引数：初期電荷[q]
/// 第6引数：ソルバータイプ
pub fn make_rlc_circuit_model(r: f64, l: f64, c: f64, init_i: f64, init_q: f64, stype: SolverType) -> SpaceStateModel {
    let inbus = vec![
        SigDef::new("v_in", "V") // 入力電圧
    ];
    let outbus = vec![
        SigDef::new("Vr", "V"),  // 抵抗の電圧
        SigDef::new("Vc", "V"),  // コンデンサの電圧
    ];

    let mut model = SpaceStateModel::new(
        inbus, outbus, 2, stype).unwrap();

        model.set_mtrx_a(&[-r / l, -1.0 / (l * c), 1.0, 0.0]).unwrap();
        model.set_mtrx_b(&[1.0 / l, 0.0]).unwrap();
        model.set_mtrx_c(&[r, 0.0, 0.0, 1.0 / c]).unwrap();
        model.set_init_state(&[init_i, init_q]).unwrap();

    model
}

#[cfg(test)]
mod sample_model_test {
    use super::*;
    
    use crate::simcore::sim_model::{sink_models::SimRecorder, source_models::StepFunc};
    use crate::simcore::sim_system::SimSystem;
    use sim_signal::signal::{SigDef};

    #[test]
    fn rlc_cirsuit_test() {
        // モデルの作成
        let mut rlc = make_rlc_circuit_model(2.0, 1e-3, 10e-6, 0.0, 0.0, SolverType::RungeKutta);

        let input = StepFunc::new(
            Bus::try_from(vec![SigDef::new("v_in", "V")]).unwrap(),
            vec![
                (0.000, 1.0, 0.001),
            ]
        ).unwrap();

        let mut scp = SimRecorder::new(
            RefBus::try_from(vec![
                SigDef::new("V_in", "V"),   // 入力
            //    SigDef::new("q", "C"),   // コンデンサの電荷
                SigDef::new("Vr", "V"),  // 抵抗の電圧
                SigDef::new("Vc", "V"),  // コンデンサの電圧
                //SigDef::new("Vl", "L"),  // インダクタの電圧
            ]).unwrap()
        );

        // 信号の接続
        rlc.interface_in().unwrap().connect_to(input.interface_out().unwrap(), 
            &["v_in"], &["v_in"]).unwrap(); // 入力電圧をrlcモデルに接続
        
        scp.interface_in().unwrap().connect_to(input.interface_out().unwrap(),
            &["v_in"], &["V_in"]).unwrap();
        scp.interface_in().unwrap().connect_to(rlc.interface_out().unwrap(), 
            &["Vr", "Vc"], &["Vr", "Vc"]).unwrap(); // スコープにrlcの出力を接続

        // システムの定義
        let mut sys = SimSystem::new(0.0, 0.01, 0.00001);

        sys.regist_model(input);
        sys.regist_model(rlc);
        
        sys.regist_recorder("scp1", scp);

        sys.run();

        sys.get_recorder("scp1").unwrap().timeplot_all(
            "test_output\\rlc_test.png", (500, 500), (3, 1)).unwrap();



    }
}

/// ボールアンドビームのサンプル
extern crate nalgebra as na;
use na::{DMatrix};
#[derive(Debug)]
pub struct BallAndBeam {
    mball: f64, // ボール重量[kg]
    jball: f64, // ボールの慣性モーメント[kg・m^2]
    jbeam: f64, // ビームの慣性モーメント[kg・,^2]
    m0: f64, // 等価質量
    inbus: RefBus, // 入力バス（モータトルク）
    outbus: Bus, // 出力バス（ボール位置、ビーム角度）
    state: DMatrix<f64>, // 状態ベクトル
}

impl BallAndBeam {
    /// ball_r: ボール半径[m], ball_weight: ボール重量[kg] ball_inertia: ボールイナーシャ beam_inertia：ビームイナーシャ
    /// init_r: ボール初期位置[m]　init_v:ボール初速度[m/s] init_theta: ビーム初期角度[deg]　init_omega: ビーム初期角速度[deg/s]
    pub fn new(ball_r: f64, ball_weight: f64, ball_inertia: f64, beam_inertia: f64, init_r: f64, init_v: f64, init_theta: f64, init_omega: f64) -> Self {        
        let mut state = DMatrix::from_element(4, 1, 0.0);
        state[0] = init_r;
        state[1] = init_v;
        state[2] = init_theta.deg2rad();
        state[3] = init_omega.deg2rad();

        Self {
            mball: ball_weight,
            jball: ball_inertia,
            jbeam: beam_inertia,
            m0: ball_weight / (ball_weight + ball_inertia / ball_r.powi(2)),
            inbus: RefBus::try_from(vec![
                        SigDef::new("trq", "Nm") // モータトルク
                    ]).unwrap(),
            outbus: Bus::try_from(vec![
                        SigDef::new("ball_r", "m"),
                        SigDef::new("ball_v", "m/s"),
                        SigDef::new("beam_t", "deg"),
                        SigDef::new("beam_w", "deg/s"),
                    ]).unwrap(),
            state: state,
        }
    }
}

impl ModelCore for BallAndBeam {
    fn initialize(&mut self, _sim_time: &SimTime) {
        //self.outbus.iter_mut().for_each(|sig| sig.set_val(0.0));
        self.outbus[0].set_val(self.state[0]);
        self.outbus[1].set_val(self.state[1]);
        self.outbus[2].set_val(self.state[2].rad2deg());
        self.outbus[3].set_val(self.state[3].rad2deg());
    }

    fn finalize(&mut self) {
        // 処理なし
    }

    fn nextstate(&mut self, sim_time: &SimTime) {
        self.rungekutta_method(sim_time.delta_t());
        //self.euler_method(sim_time.delta_t());

        //self.outbus.import_matrix(&self.state);
        self.outbus[0].set_val(self.state[0]);
        self.outbus[1].set_val(self.state[1]);
        self.outbus[2].set_val(self.state[2].rad2deg());
        self.outbus[3].set_val(self.state[3].rad2deg());
    }

    fn interface_in(&mut self) -> Option<&mut RefBus> {
        Some(&mut self.inbus)
    }

    fn interface_out(&self) -> Option<&Bus> {
        Some(&self.outbus)
    }

}

impl DEModel for BallAndBeam {
    fn set_state(&mut self, newstate: DMatrix<f64>) {
        self.state = newstate;
    }

    fn get_state(&self) -> &DMatrix<f64> {
        &self.state
    }

    fn derivative_func(&self, x: &DMatrix<f64>) -> DMatrix<f64> {
        let mut slope = DMatrix::from_element(4, 1, 0.0);
        let u = self.inbus.get_by_name("trq").unwrap().val();
        // r
        slope[0] = x[1];
        // v        
        //slope[1] = 0.0;                                                         // 抵抗力の項　符号あってる？
        slope[1] = self.m0 * G * x[2].sin() + self.m0 * x[0] * x[3] * x[3];// - self.mu * G * self.m1 * x[2].cos() - self.k * x[1] / self.m0; 
        // theta
        slope[2] = x[3];
        // omega
        let j0 = self.mball * x[0] * x[0] + self.jbeam + self.jball;
        slope[3] = (u - 2.0 * self.mball * x[0] * x[1] * x[3] + self.mball * G * x[0] * x[2].cos()) / j0;

        slope
    }
}