

/// # SubSystemモデル
/// - いくつかのモデルをひとまとまりにしてサブモデルとして定義
/// - サブモデルはSimSystemに他のモデル同様に登録することができる
/// - サブモデルは内部はSimSystemで設定しているΔtよりも短いΔtを持つことができる
/// 
use anyhow::{anyhow, Context};

use crate::simcore::{sim_model, sim_signal, sim_system};

use sim_model::model_core::{ModelCore};
use sim_signal::signal::SigDef;
use sim_signal::bus::{Bus, RefBus};

use sim_system::SimTime;


/// サブシステムモデル
pub struct SubSystem<'a> {
    inbus: RefBus, // 入力バス
    outbus: Bus, // 出力バス
    models: Vec<Box<dyn ModelCore + 'a>>, // 個々のモデルを管理するHashMap
    delta_t: f64, // 時間刻み
}

impl<'a> SubSystem<'a> {
    pub fn new(input_def: Vec<SigDef>, output_def:Vec<SigDef>, delta_t: f64) -> anyhow::Result<Self> {
        let inbus = RefBus::try_from(input_def).context(format!("SubSystemの入力バスが不正です。"))?;
        let outbus = Bus::try_from(output_def).context(format!("SubSystemの出力バスが不正です。"))?;

        Ok(Self {
            inbus: inbus,
            outbus: outbus,
            models: Vec::<Box<dyn ModelCore>>::new(),
            delta_t: delta_t
        })
    }

    /// モデルの登録
    pub fn regist_model<T>(&mut self, model: T) 
    where T: ModelCore + 'a
    {
        self.models.push(Box::new(model));
    }

    /// SubSystem内部のモデルに対してSubSystemの入力インターフェースを接続する
    pub fn connect_inbus<T:ModelCore>(&mut self, target_mdl: T, srclist: &[&str], dstlist: &[&str]) -> anyhow::Result<()> {

        Ok(())
    }

    /// SubSystem内部のモデルに対してSubSystemの出力インターフェースを接続する
    pub fn connect_outbus<T:ModelCore>(&mut self, target_mdl: T, srclist: &[&str], dstlist: &[&str]) -> anyhow::Result<()> {
        
        Ok(())
    }
}

impl<'a> ModelCore for SubSystem<'a> {
    fn initialize(&mut self, sim_time: &SimTime) {
        if self.delta_t > sim_time.delta_t() { // もしSimSystem側の刻み幅よりも大きい設定になっていたらSimSystemに合わせる
            self.delta_t = sim_time.delta_t(); 
        }

        // モデルの初期化
        self.models.iter_mut().for_each(|mdl| mdl.initialize(sim_time));
    }

    fn finalize(&mut self) {
        // モデルのファイナライズ
        self.models.iter_mut().for_each(|mdl| mdl.finalize());
    }

    fn nextstate(&mut self, sim_time: &SimTime) {
        // 各モデルを1ステップ進める
        self.models.iter_mut().for_each(|mdl| mdl.nextstate(sim_time));
    }

    fn interface_in(&mut self) -> Option<&mut RefBus> {
        Some(&mut self.inbus)
    }

    fn interface_out(&self) -> Option<&Bus> {
        Some(&self.outbus)
    }
}
