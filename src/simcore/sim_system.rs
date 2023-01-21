/// モデルを組み合わせて一つのシステムを構成する

use super::sim_model::model_core::ModelCore;
use super::sim_signal;
use sim_signal::bus::{Bus, RefBus};

/*pub trait SystemTrait {
    /// シミュレーション時間を1ステップ進める
    fn nextstate(&mut self, delta_t: f64);

    /// 入力インターフェース
    fn interface_in(&mut self) -> Option<&mut RefBus>;

    /// 出力インターフェース
    fn interface_out(&self) -> Option<&Bus>;
}*/

pub struct SimSystem<T:ModelCore> {
    models: Vec<T>, // 個々のモデルを管理するベクタ　ひとまず、この配列順で実行する。将来的には、calc_orderなどの計算順を決める配列を用意する
}

impl<T:ModelCore> SimSystem<T> {
    pub fn new() -> Self {
        Self {
            models: Vec::<T>::new(),
        }
    }

    pub fn regist_model(&mut self, model: T) {
        self.models.push(model);
    }



}
