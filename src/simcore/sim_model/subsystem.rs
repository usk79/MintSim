

/// # SubSystemモデル
/// - いくつかのモデルをひとまとまりにしてサブモデルとして定義
/// - サブモデルはSimSystemに他のモデル同様に登録することができる
/// - サブモデルは内部はSimSystemで設定しているΔtよりも短いΔtを持つことができる
/// 
use anyhow::{anyhow, Context};

use crate::simcore::{sim_model, sim_signal, sim_system};

use sim_model::model_core::{ModelCore};
use sim_signal::signal::SigDef;
use sim_signal::bus::{Bus, RefBus};

use sim_system::SimTime;

/// サブシステムモデル
pub struct SubSystem<'a> {
    inbus: RefBus, // 入力バス
    inbus_buf: Bus, // 入力バスのバッファ inbusの値をコピーしておいておき、内部のモデルがここを参照する
    outbus: Bus, // 出力バス
    outbus_buf: RefBus, // 出力バスのバッファ 内部モデルの出力をここで参照し、outbusにコピーして出力する
    models: Vec<Box<dyn ModelCore + 'a>>, // 個々のモデルを管理するHashMap
    delta_t: f64, // 時間刻み
}

impl<'a> SubSystem<'a> {
    pub fn new(input_def: Vec<SigDef>, output_def:Vec<SigDef>, delta_t: f64) -> anyhow::Result<Self> {
        let inbus = RefBus::try_from(input_def).context(format!("SubSystemの入力バスが不正です。"))?;
        let outbus = Bus::try_from(output_def).context(format!("SubSystemの出力バスが不正です。"))?;

        let inbus_def = inbus.get_sigdef();   // inbusと同じ信号定義でoutbus_bufを作成する 
        let outbus_def = outbus.get_sigdef(); // outbusと同じ信号定義でoutbus_bufを作成する

        Ok(Self {
            inbus: inbus,
            inbus_buf: Bus::try_from(inbus_def).unwrap(),
            outbus: outbus,
            outbus_buf: RefBus::try_from(outbus_def).unwrap(),
            models: Vec::<Box<dyn ModelCore>>::new(),
            delta_t: delta_t
        })
    }

    /// モデルの登録
    pub fn regist_model<T>(&mut self, model: T) 
    where T: ModelCore + 'a
    {
        self.models.push(Box::new(model));
    }

    /// SubSystem内部のモデルの入力にSubSystemの入力インターフェースを接続する
    pub fn connect_inbus<T:ModelCore>(&self, target_mdl: &mut T, srclist: &[&str], dstlist: &[&str]) -> anyhow::Result<()> {
        if let Some(target_inbus) = target_mdl.interface_in() {
            target_inbus.connect_to(&self.inbus_buf, srclist, dstlist)?;
        } else {
            return Err(anyhow!("入力インターフェースが定義されていないモデルです。信号の接続はできません。"));
        }
        Ok(())
    }

    /// SubSystem内部のモデルの出力をSubSystemの出力インターフェースに接続する
    pub fn connect_outbus<T:ModelCore>(&mut self, target_mdl: &T, srclist: &[&str], dstlist: &[&str]) -> anyhow::Result<()> {
        if let Some(target_outbus) = target_mdl.interface_out() {
            self.outbus_buf.connect_to(target_outbus, srclist, dstlist)?
        }
        Ok(())
    }
}

impl<'a> ModelCore for SubSystem<'a> {
    fn initialize(&mut self, sim_time: &SimTime) {
        if self.delta_t > sim_time.delta_t() { // もしSimSystem側の刻み幅よりも大きい設定になっていたらSimSystemに合わせる
            self.delta_t = sim_time.delta_t(); 
        }

        // モデルの初期化
        self.models.iter_mut().for_each(|mdl| mdl.initialize(sim_time));
    }

    fn finalize(&mut self) {
        // モデルのファイナライズ
        self.models.iter_mut().for_each(|mdl| mdl.finalize());
    }

    fn nextstate(&mut self, sim_time: &SimTime) {
        // 入力バスの値をバッファへコピーする
        self.inbus_buf.copy_val_from_bus(&self.inbus);
        // 各モデルを1ステップ進める
        self.models.iter_mut().for_each(|mdl| mdl.nextstate(sim_time));
        // 出力バッファの値を出力バスへコピーする
        self.outbus.copy_val_from_bus(&self.outbus_buf);
    }

    fn interface_in(&mut self) -> Option<&mut RefBus> {
        Some(&mut self.inbus)
    }

    fn interface_out(&self) -> Option<&Bus> {
        Some(&self.outbus)
    }
}


#[cfg(test)]
mod subsystem_test {
    use crate::simcore::sim_model::model_core::connect_models;
    use crate::simcore::{sim_model, sim_signal, sim_system};
    use sim_model::{de_models, controller_models, subsystem, source_models, sample_models, sink_models};
    use controller_models::PIDController;
    use subsystem::SubSystem;
    use de_models::SolverType;
    use source_models::StepFunc;
    use sample_models::BallAndBeam;
    use sink_models::SimRecorder;
    use sim_signal::signal::SigDef;
    use sim_system::SimSystem;


    fn make_controller<'a>() -> SubSystem<'a> {
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

        // モデルの接続 コントローラ1と2を接続
        connect_models(&pos_ctrl, &["target_angle"], 
                       &mut beam_ctrl, &["target_angle"]).unwrap();

        // サブシステムの作成
        let mut sys = SubSystem::new(  
            vec![
                SigDef::new("target_pos", "m"),
                SigDef::new("ball_pos", "m"),
                SigDef::new("beam_angle", "deg")
            ],
            vec![
                SigDef::new("motor_trq", "Nm")
            ],
            1.0
        ).unwrap();

        // サブシステムの入出力に内部モデルの信号を接続する
        sys.connect_inbus(&mut pos_ctrl, &["target_pos", "ball_pos"], &["target_pos", "pos"]).unwrap();
        sys.connect_inbus(&mut beam_ctrl, &["beam_angle"], &["angle"]).unwrap();

        sys.connect_outbus(&beam_ctrl, &["motor_trq"], &["motor_trq"]).unwrap();

        // サブシステムにモデルを登録する
        sys.regist_model(pos_ctrl);
        sys.regist_model(beam_ctrl);

        sys
    }

    #[test]
    fn subsystem_test() {
        // ボールアンドビームを2つのコントローラをサブシステムにまとめて実装してみる

        // Step. 1 モデルの作成
        // 目標値
        let input = StepFunc::new(
            vec![SigDef::new("target_pos", "m")],
            vec![(0.000, 0.5, 0.0)]
        ).unwrap(); // 目標位置

        // コントローラ
        let mut ctrl = make_controller();

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
            vec![
                SigDef::new("ball_pos", "m"),
                SigDef::new("ball_spd", "m/s"),
                SigDef::new("beam_angle", "deg"),
                SigDef::new("beam_omega", "deg/s"),
                SigDef::new("motor_trq", "Nm"),
            ]
        ).unwrap();

        // Step. 2 信号の接続
        // 目標値とコントローラの接続
        connect_models(
            &input,         &["target_pos"], 
            &mut ctrl,  &["target_pos"]
        ).unwrap();

        // コントローラとボールアンドビームの接続
        connect_models(
            &ctrl, &["motor_trq"], 
            &mut bab,  &["trq"]
        ).unwrap();

        // ボールアンドビームの出力をコントローラへ接続
        connect_models(
            &bab,           &["ball_r", "beam_t"], 
            &mut ctrl,  &["ball_pos", "beam_angle"]
        ).unwrap();

        // スコープに各種信号を接続
        connect_models(
            &ctrl, &["motor_trq"], 
            &mut scp,   &["motor_trq"]
        ).unwrap();

        connect_models(
            &bab,       &["ball_r", "ball_v", "beam_t", "beam_w"],
            &mut scp,   &["ball_pos", "ball_spd", "beam_angle", "beam_omega"]
        ).unwrap();

        // Step. 3 シミュレーションモデルの登録
        let mut sys = SimSystem::new(0.0, 50.0, 0.001);
        
        sys.regist_model(input);
        sys.regist_model(ctrl);
        sys.regist_model(bab);

        sys.regist_recorder("scp1", scp);

        // シミュレーション実行
        sys.run();

        // シミュレーション結果出力
        sys.get_recorder("scp1").unwrap().timeplot_all(
            "test_output\\bab_submdl.png", 
            (500, 500),
            (3, 2)
        ).unwrap();

        sys.get_recorder("scp1").unwrap().export("test_output\\bab_submdl.csv").unwrap();

    }
}