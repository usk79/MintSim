

/// # SubSystemモデル
/// - いくつかのモデルをひとまとまりにしてサブモデルとして定義
/// - サブモデルはSimSystemに他のモデル同様に登録することができる
/// - サブモデルは内部はSimSystemで設定しているΔtよりも短いΔtを持つことができる
/// 
use anyhow::{anyhow};
use std::f64::consts::{PI};

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
}
/*
impl<'a> SubSystem<'a> {
     pub fn new(input_def: Vec<SigDef>, output_def:Vec<SigDef>) -> Self {

        Self {

        }
     }
}*/

