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
use std::f64::consts::{PI};

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
    settings: Vec<(f64, f64, f64)>,
}

impl StepFunc {
    /// ## StepFuncの引数定義
    /// 1. 第1引数：Bus
    /// 1. 第2引数：settings: Vec<(init_value, final_value, step_time)>
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
            sig.set_val(self.settings[idx].0)
        });
    }

    fn finalize(&mut self) {
        // 処理なし
    }

    fn nextstate(&mut self, sim_time: &SimTime) {
        self.outbus.iter_mut().enumerate().for_each(|(idx, sig)| {
            let set = self.settings[idx];
            if sim_time.time() >= set.2 {
                sig.set_val(set.1);
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
pub struct RampFunc {
    outbus: Bus, 
    settings: Vec<(f64, f64, bool, f64, f64)>, // Vec<(init_value, final_value, start_time, slope, limit_enable)>
}

impl RampFunc {
    /// ## RampFuncの引数定義
    /// 1. 第1引数：Bus
    /// 1. 第2引数：settings: Vec<(init_value, limit_value, limit_enable, start_time, slope)>
    /// ## 注意事項
    /// Busの要素数とsettingsの要素数は等しい必要があります。
    pub fn new(outbus: Bus, settings: Vec<(f64, f64, bool, f64, f64)>) -> anyhow::Result<Self> {
        if outbus.len() != settings.len() {
            return Err(anyhow!("outbusとsettingsの要素数は一致している必要があります。\noutbus.len = {}, settings.len = {} ", outbus.len(), settings.len()))
        }

        Ok(Self {
            outbus: outbus,
            settings: settings,
        })
    }
}

impl ModelCore for RampFunc {
    fn initialize(&mut self, _sim_time: &SimTime) {
        self.outbus.iter_mut().enumerate().for_each(|(idx, sig)| {
            sig.set_val(self.settings[idx].0)
        });
    }

    fn finalize(&mut self) {
        // 処理なし
    }

    fn nextstate(&mut self, sim_time: &SimTime) {
        self.outbus.iter_mut().enumerate().for_each(|(idx, sig)| {
            let set = self.settings[idx];
            let delta = set.4 * sim_time.delta_t(); // 1ステップ当たりの増加量
            if sim_time.time() >= set.3 {
                let mut val = sig.val() + delta;
                if set.2 == true && delta >= 0.0 { // 上限が有効であれば上限を設定する
                    val = val.min(set.1);
                } else if set.2 == true && delta < 0.0 {
                    val = val.max(set.1);
                }
                sig.set_val(val);
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

/// # 波の関数の種類定義
#[derive(Debug)]
enum WaveFuncType {
    Sin,        // 正弦波
    Triangle,   // 三角波
    // 矩形波（Duty50の矩形波のみ）
}

/// # 波の関数の設定用構造体
/// 振幅・位相・周期（周波数）・オフセットをメンバ持つ構造体。
/// 正弦波、三角波、矩形波などの周期関数の設定で共通使用する
#[derive(Debug)]
pub struct WaveFuncSetting {
    fn_type: WaveFuncType,
    amplitude: f64, // 振幅
    phase: f64,  // 位相[rad]
    period: f64, // 周期[s]
    offset: f64, // オフセット
}

/// 正弦波関数モデル
#[derive(Debug)]
pub struct SinFunc {
    outbus: Bus,
    settings: Vec<WaveFuncSetting>,
}

impl SinFunc {
    /// ## SinFuncの引数定義
    /// 1. 第1引数：Bus
    /// 1. 第2引数：settings: Vec<WafeFuncSetting>
    pub fn new(outbus: Bus, settings: Vec<WaveFuncSetting>) -> anyhow::Result<Self> {
        if outbus.len() != settings.len() {
            return Err(anyhow!("outbusとsettingsの要素数は一致している必要があります。\noutbus.len = {}, settings.len = {} ", outbus.len(), settings.len()));
        }

        Ok(Self{
            outbus: outbus,
            settings: settings,
        })
    }
   
}

/// 振幅1 周期2πの波関数定義
fn wave_func(set: &WaveFuncSetting, t: f64) -> f64 {
    match set.fn_type {
        WaveFuncType::Sin => t.sin(),
        WaveFuncType::Triangle => t.cos(),
    }
}

impl ModelCore for SinFunc {
    fn initialize(&mut self, _sim_time: &SimTime) {
        self.outbus.iter_mut().enumerate().for_each(|(idx, sig)|{
            let set = &self.settings[idx];
            let t = set.phase;
            let val = set.amplitude * t.sin() + set.offset;
            sig.set_val(val);
        })
    }

    fn finalize(&mut self) {
        // 処理なし
    }

    fn nextstate(&mut self, sim_time: &SimTime) {
        self.outbus.iter_mut().enumerate().for_each(|(idx, sig)| {
            let set = &self.settings[idx];
            let t = 2.0 * sim_time.time() / set.period * PI + set.phase;
            let val = set.amplitude * wave_func(set, t) + set.offset;
            sig.set_val(val);
        })
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
    use crate::simcore::sim_common::UnitTrans;
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

    #[test]
    fn ramp_func_test() {
        let sf = RampFunc::new(
            Bus::try_from(vec![
                SigDef::new("rf1", "Nm"),
                SigDef::new("rf2", "A"),
                SigDef::new("rf3", "Nm"),
                SigDef::new("rf4", "A"),
            ]).unwrap(),
            vec![
                (0.5, 1.5, true, 0.2, 2.0),
                (0.5, 0.0, false, 0.3, 2.0),
                (-0.5, -1.5, true, 0.2, -2.0),
                (-0.5, 0.0, false, 0.3, -2.0),
            ]
        ).unwrap();

        let mut scp = SimRecorder::new(
            RefBus::try_from(vec![
                SigDef::new("Scp_rf1", "Nm"),
                SigDef::new("Scp_rf2", "A"),
                SigDef::new("Scp_rf3", "Nm"),
                SigDef::new("Scp_rf4", "A"),
            ]).unwrap()
        );

        scp.interface_in().unwrap().connect_to(sf.interface_out().unwrap(), 
            &["rf1", "rf2", "rf3", "rf4"], 
            &["Scp_rf1", "Scp_rf2", "Scp_rf3", "Scp_rf4"],
        ).unwrap();

        let mut sys = SimSystem::new(0.0, 1.0, 0.01);

        sys.regist_model(sf);
        sys.regist_recorder("scp1", scp);

        sys.run();

        sys.get_recorder("scp1").unwrap().timeplot_all(
            "test_output\\ramp_func.png", 
            (500, 500),
            (2, 2)
        ).unwrap();

    }

    #[test]
    #[should_panic]
    fn ramp_func_panic_test() {
        let _sf = RampFunc::new(
            Bus::try_from(vec![
                SigDef::new("Rf1", "Nm"),
                SigDef::new("Rf2", "A"),
            ]).unwrap(),
            vec![
                (0.5, 1.5, true, 0.2, 2.0),
                (0.5, 0.0, false, 0.3, 2.0),
                (0.5, -1.5, true, 0.2, -2.0),
                (0.5, 0.0, false, 0.3, -2.0),]
        ).unwrap();
    }

    #[test]
    fn sin_func_test() {
        let sinf = SinFunc::new(
            Bus::try_from(vec![
                SigDef::new("Sin1", "Nm"),
                SigDef::new("Sin2", "A"),
            ]).unwrap(),
            vec![
                WaveFuncSetting {
                    fn_type: WaveFuncType::Sin,
                    amplitude: 2.0,
                    phase: 0.0,
                    period: 0.5,
                    offset: 0.0,
                },
                WaveFuncSetting {
                    fn_type: WaveFuncType::Sin,
                    amplitude: 2.0,
                    phase: 90.0.deg2rad(),
                    period: 0.3,
                    offset: 1.0,
                },
            ]
        ).unwrap();

        let mut scp = SimRecorder::new(
            RefBus::try_from(vec![
                SigDef::new("Scp_rf1", "Nm"),
                SigDef::new("Scp_rf2", "A"),
            ]).unwrap()
        );

        scp.interface_in().unwrap().connect_to(sinf.interface_out().unwrap(),
            &["Sin1", "Sin2"],
            &["Scp_rf1", "Scp_rf2"] 
        ).unwrap();

        let mut sys = SimSystem::new(0.0, 1.0, 0.01);

        sys.regist_model(sinf);
        sys.regist_recorder("scp1", scp);

        sys.run();

        sys.get_recorder("scp1").unwrap().timeplot_all(
            "test_output\\sin_func.png", 
            (500, 500),
            (2, 1)
        ).unwrap();


    }

    #[test]
    #[should_panic]
    fn sin_func_panic_test() {
        let _sinf = SinFunc::new(
            Bus::try_from(vec![
                SigDef::new("Sin1", "Nm"),
                SigDef::new("Sin2", "A"),
            ]).unwrap(),
            vec![
                WaveFuncSetting {
                    fn_type: WaveFuncType::Sin,
                    amplitude: 2.0,
                    phase: 0.0,
                    period: 0.5,
                    offset: 0.0,
                },
            ]
        ).unwrap();

    }
}