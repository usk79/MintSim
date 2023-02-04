use crate::simcore::sim_signal::signal::SigTrait;

/// # Sourceモデル
/// Sourceモデルには、下記のモデルを実装する
/// 
/// - 定数モデル
/// - Step関数
/// - Ramp関数
/// - 三角波関数
/// - 矩形波
/// - Lookup（CSVファイル読み込み）　時間に足りない分の選択肢（0にするか、繰り返すか）　時間の間は線形補完
use anyhow::{anyhow};

use super::model_core::{ModelCore};

use super::super::sim_signal;

use sim_signal::bus::{Bus, RefBus};
use super::super::sim_system;
use sim_system::SimTime;

//　モデルを追加した時に実装するメソッド(ModelCoreトレイト)

// /// 初期化処理
// fn initialize(&mut self, delta_t: f64);

// /// シミュレーション時間を1ステップ進める
// fn nextstate(&mut self, sim_time: f64);

// /// 終了処理
// fn finalize(&mut self);

// /// 入力インターフェース
// fn interface_in(&mut self) -> Option<&mut RefBus>;

// /// 出力インターフェース
// fn interface_out(&self) -> Option<&Bus>;

/// 定数モデル
#[derive(Debug)]
pub struct ConstantFunc {
    outbus: Bus,
}

impl ConstantFunc {
    pub fn new(mut outbus: Bus, values: &[f64]) -> anyhow::Result<Self> {
        if outbus.len() != values.len() {
            return Err(anyhow!("outbusとvaluesの要素数が異なります。outbus長:{}, values長:{}", outbus.len(), values.len()));
        }

        outbus.iter_mut().enumerate().for_each(|(i, sig)| sig.set_val(values[i]));

        Ok(Self {
            outbus: outbus,
        })
    }
}

impl ModelCore for ConstantFunc {
    fn initialize(&mut self, _sim_time: &SimTime) {
        
    }

    fn finalize(&mut self) {
        // 処理なし
    }

    fn nextstate(&mut self, _sim_time: &SimTime) {
        // 処理なし
    }

    fn interface_in(&mut self) -> Option<&mut RefBus> {
        None
    }

    fn interface_out(&self) -> Option<&Bus> {
        Some(&self.outbus)
    }
}

/// # STEP関数モデル
/// 指定した時刻未満の時は初期値を出力し、指定時刻以降は最終値を出力する
#[derive(Debug)]
pub struct StepFunc {
    outbus: Bus,
    settings: Vec<(f64, f64, f64)>, // (step_time, init_value, final_value)
}

impl StepFunc {
    /// ## StepFuncの引数定義
    /// 1. 第1引数：Bus
    /// 1. 第2引数：settings: Vec<(step_time, init_value, final_value)>
    /// ## 注意事項
    /// Busの要素数とsettingsの要素数は等しい必要があります。
    pub fn new(outbus: Bus, settings: Vec<(f64, f64, f64)>) -> anyhow::Result<Self> {
        if outbus.len() != settings.len() {
            return Err(anyhow!("outbusとsettingsの要素数は一致している必要があります。\noutbus.len = {}, settings.len = {} ", outbus.len(), settings.len()))
        }

        Ok(Self {
            outbus: outbus,
            settings: settings,
        })
    }
}

impl ModelCore for StepFunc {
    fn initialize(&mut self, _sim_time: &SimTime) {
        self.outbus.iter_mut().enumerate().for_each(|(idx, sig)| {
            sig.set_val(self.settings[idx].1)
        });
    }

    fn finalize(&mut self) {
        // 処理なし
    }

    fn nextstate(&mut self, sim_time: &SimTime) {
        self.outbus.iter_mut().enumerate().for_each(|(idx, sig)| {
            let set = self.settings[idx];
            if sim_time.time() >= set.0 {
                sig.set_val(set.2);
            }
        });
    }

    fn interface_in(&mut self) -> Option<&mut RefBus> {
        None
    }

    fn interface_out(&self) -> Option<&Bus> {
        Some(&self.outbus)
    }
}

/// Ramp関数モデル
/// Ramp関数は初期値、増加開始時刻、増加の傾きによって決まる関数
#[derive(Debug)]
struct RampFunc {
    outbus: Bus, 
    start_time: f64,    // 増加開始時刻
    slope: f64,         // 1秒あたりの傾き
    init_value: f64,    // 初期値
    final_value: f64,   // 最終値
    limit_enable: bool, // 最終値の有効/無効　有効時はfinal_valueで頭打ち、無効時は上限なしに増加を継続
}

impl RampFunc {
    pub fn new(outbus: Bus, start_time: f64, slope: f64, init_value: f64, final_value: f64, limit_enable: bool) -> Self {
        Self {
            outbus: outbus,
            start_time: start_time,
            slope: slope,
            init_value: init_value,
            final_value: final_value,
            limit_enable: limit_enable,
        }
    }
}

impl ModelCore for RampFunc {
    fn initialize(&mut self, _sim_time: &SimTime) {
        self.outbus.set_all(self.init_value);
    }

    fn finalize(&mut self) {
        // 処理なし
    }

    fn nextstate(&mut self, sim_time: &SimTime) {
        if sim_time.time() >= self.start_time {
            let delta = self.slope * sim_time.delta_t(); // 1ステップ当たりの傾きを求める
            let val = self.outbus[0].val() + delta;
            self.outbus.set_all(val)
        }
    }

    fn interface_in(&mut self) -> Option<&mut RefBus> {
        None
    }

    fn interface_out(&self) -> Option<&Bus> {
        Some(&self.outbus)
    }
}

#[cfg(test)]
mod source_model_test {
    use super::*;
    
    use crate::simcore::sim_model::sink_models::SimRecorder;
    use crate::simcore::sim_system::SimSystem;
    use sim_signal::signal::{SigDef};

    #[test]
    fn const_test() {
        let con = ConstantFunc::new(
            Bus::try_from(vec![
                SigDef::new("Con1", "Nm"),
                SigDef::new("Con2", "A"),
            ]).unwrap(),
            &[0.0, 1.0]
        ).unwrap();

        assert_eq!(con.interface_out().unwrap()[0].val(), 0.0);
        assert_eq!(con.interface_out().unwrap()[1].val(), 1.0);    
    }

    #[test]
    #[should_panic]
    fn const_panic_test() {
        let _con = ConstantFunc::new(
            Bus::try_from(vec![
                SigDef::new("Con1", "Nm"),
                SigDef::new("Con2", "A"),
            ]).unwrap(),
            &[0.0, 1.0, 3.0]
        ).unwrap();
    }

    #[test]
    fn step_func_test() {
        let sf = StepFunc::new(
            Bus::try_from(vec![
                SigDef::new("st1", "Nm"),
                SigDef::new("st2", "A"),
            ]).unwrap(),
            vec![
                (0.5, 0.0, 1.0),
                (0.3, 1.0, -1.0),
            ]
        ).unwrap();

        let mut scp = SimRecorder::new(
            RefBus::try_from(vec![
                SigDef::new("Scp_st1", "Nm"),
                SigDef::new("Scp_st2", "A"),
            ]).unwrap()
        );

        scp.interface_in().unwrap().connect_to(sf.interface_out().unwrap(), 
            &["st1", "st2"], 
            &["Scp_st1", "Scp_st2"],
        ).unwrap();

        let mut sys = SimSystem::new(0.0, 1.0, 0.01);

        sys.regist_model(sf);
        sys.regist_recorder("scp1", scp);

        sys.run();

        sys.get_recorder("scp1").unwrap().timeplot_all(
            "test_output\\step_func.png", 
            (500, 500),
            (2, 1)
        ).unwrap();
    }
/*
    #[test]
    fn step_func_test_old() {
        
        let stbus = Bus::try_from(vec![
            SigDef::new("step", "-"),
        ]).unwrap();

        let st = StepFunc::new(stbus, 0.3, 0.2, 0.5);
        
        let mut scope_bus = RefBus::try_from(vec![
            SigDef::new("step", "-"),
        ]).unwrap();

        scope_bus.connect_to(st.interface_out().unwrap(), &["step"], &["step"]).unwrap();
        let scope = SimRecorder::new(scope_bus);

        let mut sys = SimSystem::new(0.0, 1.0, 0.01);
        
        sys.regist_model(st);
        sys.regist_recorder("rcd1", scope);
        
        sys.run(); // sysに名前から中のモデルにアクセスできるようにハッシュマップをもたせる

        sys.get_recorder("rcd1").unwrap().timeplot_all(
            "test_output\\step_func.png", 
            (500, 500),
            (1, 1)
        ).unwrap();
        

    }
*/
    #[test]
    #[should_panic]
    fn step_func_panic_test() {
        let _sf = StepFunc::new(
            Bus::try_from(vec![
                SigDef::new("Sf1", "Nm"),
                SigDef::new("Sf2", "A"),
            ]).unwrap(),
            vec![
                (0.5, 0.0, 1.0),
                (0.3, 1.0, 1.5),
                (0.3, 1.0, 1.5)]
        ).unwrap();
    }
}