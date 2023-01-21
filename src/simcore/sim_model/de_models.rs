/// # DEモデル
/// DEモデルには、下記のモデルを実装する

/// - 状態空間モデル
/// - 微分方程式モデル
/// - 伝達関数モデル
/// - 積分器モデル

use super::model_core::{ModelCore};

use super::super::sim_signal;
use sim_signal::signal::{SigDef, SigTrait};

use sim_signal::bus::{Bus, RefBus};
