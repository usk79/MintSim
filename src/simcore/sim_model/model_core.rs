
use crate::simcore::{sim_signal, sim_system};
use sim_signal::bus::{Bus, RefBus};

use sim_system::SimTime;

use anyhow::{anyhow};

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

/// 信号の接続
pub fn connect_models<S: ModelCore, D:ModelCore>(srcmodel: &S, srclist: &[&str], dstmodel: &mut D, dstlist: &[&str]) -> anyhow::Result<()>{
    if let Some(srcbus) = srcmodel.interface_out() {
        
        if let Some(inbus) = dstmodel.interface_in() {
            inbus.connect_to(srcbus, srclist, dstlist)?
        } else {
            return Err(anyhow!("入力インターフェースが定義されていないモデルです。信号の接続はできません。"));
        }

    } else {
        return Err(anyhow!("引数に指定されているモデルは出力インターフェースが定義されていないモデルです。信号の接続はできません。"));
    }

    Ok(())
}