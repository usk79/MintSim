
use super::super::sim_signal;
use sim_signal::bus::{Bus, RefBus};


pub trait ModelCore {
    /// シミュレーション時間を1ステップ進める
    fn nextstate(&mut self, delta_t: f64);

    /// 入力インターフェース
    fn interface_in(&mut self) -> Option<&mut RefBus>;

    /// 出力インターフェース
    fn interface_out(&self) -> Option<&Bus>;
}
