
use super::super::sim_signal;
use sim_signal::bus::{Bus, RefBus};


pub trait ModelCore {
    /// 初期化処理
    fn initialize(&mut self, delta_t: f64);

    /// シミュレーション時間を1ステップ進める
    fn nextstate(&mut self, sim_time: f64);

    /// 終了処理
    fn finalize(&mut self);

    /// 入力インターフェース
    fn interface_in(&mut self) -> Option<&mut RefBus>;

    /// 出力インターフェース
    fn interface_out(&self) -> Option<&Bus>;
}
