/// # Sourceモデル
/// Sourceモデルには、下記のモデルを実装する
/// 
/// - 定数モデル
/// - Step関数
/// - Ramp関数
/// - 三角波関数
/// - 矩形波
/// - Lookup（CSVファイル読み込み）　時間に足りない分の選択肢（0にするか、繰り返すか）　時間の間は線形補完

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
pub struct Constant {
    outbus: Bus,
}

impl Constant {
    pub fn new(outbus: Bus) -> Self {
        Self {
            outbus: outbus,
        }
    }
}

impl ModelCore for Constant {
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

/// STEP関数モデル
pub struct StepFunc {
    outbus: Bus,
    step_time: f64, // ステップする時刻
    init_value: f64, // 初期値
    final_value: f64, // 最終値
}

impl StepFunc {
    pub fn new(outbus: Bus, step_time: f64, init_value:f64, final_value: f64) -> Self {
        Self {
            outbus: outbus,
            step_time: step_time,
            init_value: init_value,
            final_value: final_value,
        }
    }
}

impl ModelCore for StepFunc {
    fn initialize(&mut self, _sim_time: &SimTime) {
        self.outbus.set_all(self.init_value);
    }

    fn finalize(&mut self) {
        // 処理なし
    }

    fn nextstate(&mut self, sim_time: &SimTime) {
        if sim_time.time() >= self.step_time {
            self.outbus.set_all(self.final_value);
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
    fn step_func_test() {
        
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
        sys.regist_model(scope);
        
        sys.run(); // sysに名前から中のモデルにアクセスできるようにハッシュマップをもたせる

        

    }
}