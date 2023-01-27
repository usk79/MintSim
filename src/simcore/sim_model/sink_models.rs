/// # Sinkモデル
/// Sinkモデルには、下記のモデルを実装する
/// 
/// - -終端モデル ?　いらんか
/// - Scopeモデル
use super::model_core::{ModelCore};

use super::super::sim_signal;
use sim_signal::signal::{SigDef, SigTrait};

use sim_signal::bus::{Bus, RefBus};

use super::super::sim_system;
use sim_system::SimTime;