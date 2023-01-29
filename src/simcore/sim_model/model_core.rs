
use super::super::sim_signal;
use sim_signal::bus::{Bus, RefBus};
use super::super::sim_system;
use sim_system::SimTime;

pub const DEFAULT_DELTA_T: f64  = 0.1;

pub trait ModelCore {
    /// 初期化処理
    fn initialize(&mut self, sim_time: &SimTime);

    /// シミュレーション時間を1ステップ進める
    fn nextstate(&mut self, sim_time: &SimTime);

    /// 終了処理
    fn finalize(&mut self);

    /// 入力インターフェース
    fn interface_in(&mut self) -> Option<&mut RefBus>;

    /// 出力インターフェース
    fn interface_out(&self) -> Option<&Bus>;
}
