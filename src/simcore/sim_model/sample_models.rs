/// # Sampleモデル
/// Sampleモデルには、下記のモデルを実装する
/// 
/// - RLC回路
/// - ボールアンドビーム
use super::model_core::{ModelCore};
use super::de_models::{SpaceStateModel, SolverType};

use super::super::sim_signal;
use sim_signal::signal::{SigDef, SigTrait};

use sim_signal::bus::{Bus, RefBus};

use super::super::sim_system;
use sim_system::SimTime;


/// RLC直列回路の状態空間モデルを生成する関数（サンプル）
/// https://qiita.com/code0327/items/423b0f0380e8c64f3580 数値はここを参考
/// 第1引数：抵抗[Ω]
/// 第2引数：インダクタンス[H]
/// 第3引数：キャパシタンス[C]
/// 第4引数：初期電流[A]
/// 第5引数：初期電荷[q]
/// 第6引数：ソルバータイプ
pub fn make_rlc_circuit_model(r: f64, l: f64, c: f64, init_i: f64, init_q: f64, stype: SolverType) -> SpaceStateModel {
    let inbus = RefBus::try_from(vec![
        SigDef::new("v_in", "V") // 入力電圧
    ]).unwrap();
    let outbus = Bus::try_from(vec![
        SigDef::new("Vr", "V"),  // 抵抗の電圧
        SigDef::new("Vc", "V"),  // コンデンサの電圧
    ]).unwrap();

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
    use crate::simcore::sim_common::UnitTrans;
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

struct BallAndBeam {
    _rball: f64, // ボールの半径[m]
    mball: f64, // ボール重量[kg]
    jball: f64, // ボールの慣性モーメント[kg・m^2]
    jbeam: f64, // ビームの慣性モーメント[kg・,^2]
    _mu: f64, // ボールの転がり抵抗係数[-]
    _k: f64, // 空気抵抗係数[N/(m/s)^2]
    _m0: f64, // 途中計算
    m1: f64, // 途中計算
    inbus: RefBus, // 入力バス（モータトルク）
    outbus: Bus, // 出力バス（ボール位置、ビーム角度）
    
}