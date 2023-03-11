//extern crate anyhow;

/// #ToDo
/// - 実行前に未接続信号の警告を出せるようにする
/// - 信号の接続（デフォルト実装） ⇒　今後実装先を考える（今のところSimSystemがよいかと: さらにconnectするときにモデルも登録するようにすれば登録忘れ防止にになる

pub mod simcore;

#[cfg(test)]
mod tests {
    use crate::simcore::sim_model::model_core::{connect_models};
    use crate::simcore::sim_model::{
            sample_models::BallAndBeam, 
            source_models::StepFunc,
            controller_models::PIDController,
            sink_models::SimRecorder,
            de_models::SolverType,
    };
    use crate::simcore::sim_signal::{bus, signal};
    use crate::simcore::sim_system::SimSystem;
    use bus::{*};
    use signal::{*};

    //use super::simcore::sim_signal::bus::{Bus};
    //use super::simcore::sim_signal::signal::{Signal};

    #[test]
    fn ball_and_beam_test() {
        // Step. 1 モデルの作成
        // 目標値
        let input = StepFunc::new(
            Bus::try_from(vec![SigDef::new("target_pos", "m")]).unwrap(),
            vec![(0.000, 0.5, 0.0)]
        ).unwrap(); // 目標位置

        // PIDコントローラ1：位置制御
        let mut pos_ctrl = PIDController::new(
            vec![SigDef::new("target_pos", "m"), SigDef::new("pos", "m")],
            vec![SigDef::new("target_angle", "deg")],
            (10.0, 0.1, 8.0),
            (-10.0, 10.0),
            SolverType::RungeKutta,
        ).unwrap();

        // PIDコントローラ2：ビーム角度制御（モータ出力）
        let mut beam_ctrl = PIDController::new(
            vec![SigDef::new("target_angle", "deg"), SigDef::new("angle", "deg")],
            vec![SigDef::new("motor_trq", "Nm")],
            (1.2, 0.05, 0.3),
            (-5.0, 5.0),
            SolverType::RungeKutta,
        ).unwrap();

        // ボールアンドビームモデル
        let rball = 0.1; // 10[cm]
        let mball = 0.1; // 100[g]

        let mbeam = 1.0; // 1.0[kg]
        let lbeam = 3.0; // 1[m] ビームの長さ
        let wbeam = 0.05; // 5[cm]　ビームの幅

        let mut bab = BallAndBeam::new(
            rball,
            mball, 
            2.0 / 5.0 * mball * rball * rball,
            mbeam * (lbeam * lbeam + wbeam * wbeam) / 12.0,
            0.0,
            0.0,
            0.0,
            0.0
        );

        

        // スコープの設定
        let mut scp = SimRecorder::new(
            RefBus::try_from(vec![
                SigDef::new("ball_pos", "m"),
                SigDef::new("ball_spd", "m/s"),
                SigDef::new("beam_angle", "deg"),
                SigDef::new("beam_omega", "deg/s"),
                SigDef::new("motor_trq", "Nm"),
                SigDef::new("target_angle", "deg"),
            ]).unwrap()
        );

        // Step. 2 信号の接続
        // 目標値とPIDコントローラ1段目の接続
        connect_models(
            &input,         &["target_pos"], 
            &mut pos_ctrl,  &["target_pos"]
        ).unwrap();

        // 1段目コントローラと2段目コントローラを接続
        connect_models(
            &pos_ctrl,      &["target_angle"], 
            &mut beam_ctrl, &["target_angle"]
        ).unwrap();

        // 2段目コントローラとボールアンドビームモデルを接続
        connect_models(
            &beam_ctrl, &["motor_trq"], 
            &mut bab,  &["trq"]
        ).unwrap();

        // ボールアンドビームの出力を1段目コントローラへ接続
        connect_models(
            &bab,           &["ball_r"], 
            &mut pos_ctrl,  &["pos"]
        ).unwrap();

        // ボールアンドビームの出力を2段目コントローラへ接続
        connect_models(
            &bab,           &["beam_t"], 
            &mut beam_ctrl, &["angle"],
        ).unwrap();

        // スコープに各種信号を接続
        connect_models(
            &pos_ctrl,  &["target_angle"],
            &mut scp,   &["target_angle"]
        ).unwrap();

        connect_models(
            &beam_ctrl, &["motor_trq"], 
            &mut scp,   &["motor_trq"]
        ).unwrap();

        connect_models(
            &bab,       &["ball_r", "ball_v", "beam_t", "beam_w"],
            &mut scp,   &["ball_pos", "ball_spd", "beam_angle", "beam_omega"]
        ).unwrap();

        // Step. 3 シミュレーションモデルの登録
        let mut sys = SimSystem::new(0.0, 50.0, 0.001);

        sys.regist_model(input);
        sys.regist_model(pos_ctrl);
        sys.regist_model(beam_ctrl);
        sys.regist_model(bab);

        sys.regist_recorder("scp1", scp);

        // シミュレーション実行
        sys.run();

        // シミュレーション結果出力
        sys.get_recorder("scp1").unwrap().timeplot_all(
            "test_output\\bab.png", 
            (500, 500),
            (3, 2)
        ).unwrap();

        sys.get_recorder("scp1").unwrap().export("test_output\\bab.csv").unwrap();


    }

    #[test]
    fn beam_test() {
        // ボール無でビームの角度をコントロールするテスト (ボールの速度項を0にすること)

        // Step. 1 モデルの作成
        // 目標値
        let input = StepFunc::new(
            Bus::try_from(vec![SigDef::new("target_angle", "deg")]).unwrap(),
            vec![(0.000, 0.0, 0.0)]
        ).unwrap(); // 目標位置

        // PIDコントローラ2：ビーム角度制御（モータ出力）
        let mut beam_ctrl = PIDController::new(
            vec![SigDef::new("target_angle", "deg"), SigDef::new("angle", "deg")],
            vec![SigDef::new("motor_trq", "Nm")],
            (1.2, 0.05, 0.3),
            (-5.0, 5.0),
            SolverType::RungeKutta,
        ).unwrap();

        // ボールアンドビームモデル
        let rball = 0.1; // 10[cm]
        let mball = 0.1; // 0[g]

        let mbeam = 1.0; // 1.0[kg]
        let lbeam = 3.0; // 1[m] ビームの長さ
        let wbeam = 0.05; // 5[cm]　ビームの幅

        let mut bab = BallAndBeam::new(
            rball,
            mball, 
            2.0 / 5.0 * mball * rball * rball,
            mbeam * (lbeam * lbeam + wbeam * wbeam) / 12.0,
            0.0,
            0.0,
            -5.0,
            0.0
        );

        // スコープの設定
        let mut scp = SimRecorder::new(
            RefBus::try_from(vec![
                SigDef::new("ball_pos", "m"),
                SigDef::new("ball_spd", "m/s"),
                SigDef::new("beam_angle", "deg"),
                SigDef::new("beam_omega", "deg/s"),
                SigDef::new("motor_trq", "Nm"),
                SigDef::new("target_angle", "deg"),
            ]).unwrap()
        );

        // Step. 2 信号の接続
        // 目標値とPIDコントローラ1段目の接続
        connect_models(
            &input,         &["target_angle"], 
            &mut beam_ctrl, &["target_angle"]
        ).unwrap();

        // 2段目コントローラとボールアンドビームモデルを接続
        connect_models(
            &beam_ctrl, &["motor_trq"], 
            &mut bab,  &["trq"]
        ).unwrap();

        // ボールアンドビームの出力を2段目コントローラへ接続
        connect_models(
            &bab,           &["beam_t"], 
            &mut beam_ctrl, &["angle"],
        ).unwrap();

        // スコープに各種信号を接続
        connect_models(
            &beam_ctrl, &["motor_trq"], 
            &mut scp,   &["motor_trq"]
        ).unwrap();

        connect_models(
            &bab,       &["ball_r", "ball_v", "beam_t", "beam_w"],
            &mut scp,   &["ball_pos", "ball_spd", "beam_angle", "beam_omega"]
        ).unwrap();

        connect_models(
            &input,  &["target_angle"],
            &mut scp,   &["target_angle"]
        ).unwrap();

        // Step. 3 シミュレーションモデルの登録
        let mut sys = SimSystem::new(0.0, 3.0, 0.001);

        sys.regist_model(input);
        sys.regist_model(beam_ctrl);
        sys.regist_model(bab);

        sys.regist_recorder("scp1", scp);

        // シミュレーション実行
        sys.run();

        // シミュレーション結果出力
        sys.get_recorder("scp1").unwrap().timeplot_all(
            "test_output\\beam.png", 
            (500, 500),
            (3, 2)
        ).unwrap();

        sys.get_recorder("scp1").unwrap().export("test_output\\beam.csv").unwrap();


    }
}
