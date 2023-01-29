/// # Sampleモデル
/// Sampleモデルには、下記のモデルを実装する
/// 
/// - RLC回路
/// - ボールアンドビーム
use super::model_core::{ModelCore};

use super::super::sim_signal;
use sim_signal::signal::{SigDef, SigTrait};

use sim_signal::bus::{Bus, RefBus};

use super::super::sim_system;
use sim_system::SimTime;