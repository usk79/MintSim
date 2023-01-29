/// # Sinkモデル
/// Sinkモデルには、下記のモデルを実装する
/// 
/// - Recorderモデル
use super::model_core::{ModelCore};

use super::super::sim_signal;
use sim_signal::signal::{SigDef, SigTrait};

use sim_signal::bus::{Bus, RefBus};

use super::super::sim_system;
use sim_system::SimTime;

#[derive(Debug)]
pub struct SimRecorder {
    timedata: Vec<f64>,     // 時刻情報保管用
    storage: Vec<Vec<f64>>, // データストレージ
    signum: usize,
    input_bus: RefBus, 
}

impl SimRecorder {
    pub fn new(inbus: RefBus) -> Self {
        Self {
            timedata: Vec::new(),
            storage: Vec::new(),
            signum: inbus.len(),
            input_bus: inbus,
        }
    }
}

impl ModelCore for SimRecorder {
    fn initialize(&mut self, sim_time: &SimTime) {
        let stepnum = sim_time.step_num();

        self.timedata = Vec::with_capacity(stepnum);
        self.storage = (0..self.signum).map(|_| Vec::with_capacity(stepnum) ).collect::<Vec<Vec<f64>>>();

        self.timedata.push(sim_time.start_time()); // 初期時間を設定する
        // ここに各初期値を設定する　ひとまず0埋めでOK
    }

    fn finalize(&mut self) {
        // 処理なし
    }

    fn nextstate(&mut self, sim_time: &SimTime) {
        // 処理なし
        self.timedata.push(sim_time.time());
        
        self.input_bus.iter().enumerate().for_each(|(idx, sig)| {
            self.storage[idx].push(sig.val());
        })

    }

    fn interface_in(&mut self) -> Option<&mut RefBus> {
        Some(&mut self.input_bus)
    }

    fn interface_out(&self) -> Option<&Bus> {
        None
    }
}
