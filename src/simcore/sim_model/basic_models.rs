/// # basicモデル
/// basicモデルには、下記のモデルを実装する
/// 
/// - step delayモデル
/// - time delayモデル
/// - unit delayモデル

use super::model_core::{ModelCore};

use super::super::sim_signal;
use sim_signal::signal::{SigDef, SigTrait};

use sim_signal::bus::{Bus, RefBus};

use super::super::sim_system;
use sim_system::SimTime;