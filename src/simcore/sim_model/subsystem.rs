

/// # SubSystemモデル
/// - いくつかのモデルをひとまとまりにしてサブモデルとして定義
/// - サブモデルはSimSystemに他のモデル同様に登録することができる
/// - サブモデルは内部はSimSystemで設定しているΔtよりも短いΔtを持つことができる
/// 
use anyhow::{anyhow};
use std::f64::consts::{PI};

use super::model_core::{ModelCore};

use super::super::sim_signal;
use sim_signal::signal::SigTrait;
use sim_signal::bus::{Bus, RefBus};
use super::super::sim_system;
use sim_system::SimTime;


/// サブシステムモデル
pub struct SubSystem<'a> {
    inbus: RefBus, // 入力バス
    outbus: Bus, // 出力バス

    models: Vec<Box<dyn ModelCore + 'a>>, // 個々のモデルを管理するHashMap　
}

impl<'a> SubSystem<'a> {
     pub fn new(input_def: Vec<SigDef>, output_def:Vec<SigDef>) -> Self {
        
        Self {

        }
     }
}

