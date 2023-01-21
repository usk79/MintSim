/// # Sourceモデル
/// Sourceモデルには、下記のモデルを実装する
/// 
/// - 定数モデル
/// - Step関数
/// - Ramp関数
/// - Sin関数
/// - 三角波関数

use super::model_core::{ModelCore};

use super::super::sim_signal;
use sim_signal::signal::{SigDef, SigTrait};

use sim_signal::bus::{Bus, RefBus};