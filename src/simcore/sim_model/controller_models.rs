/// # controllerモデル
/// Controllerモデルには、下記のモデルを実装する
/// 
/// - PID制御モデル

use anyhow::{anyhow, Context};

use crate::simcore::{sim_model, sim_signal, sim_system};
use sim_model::model_core::{ModelCore};
use sim_model::de_models::{Integrator, SolverType};

use sim_signal::signal::{SigDef, SigTrait};
use sim_signal::bus::{Bus, RefBus};

use sim_system::SimTime;

use super::super::sim_common::Saturation;

/// PIDコントローラモデル
#[derive(Debug, Clone)]
pub struct PIDController {
    integrator: Integrator, // 積分器
    u_old: f64, // 入力前回値（微分用）
    input_bus: RefBus, // 必ず2要素で使用する（1要素目：目標値、2要素目：現在値)
    output_bus: Bus, // 必ず1要素で使用する
    error_bus: Bus, // 誤差（目標値 - 現在値)
    gain: (f64, f64, f64), // PIDゲイン配列 (P, I, D)
    minmax: (f64, f64), // 出力の上下限 (min, max)
}

impl PIDController {
    /// PID 入力バス定義：第1要素目⇒目標値、第2要素⇒現在値
    pub fn new(input_def: Vec<SigDef>, output_def: Vec<SigDef>, gain: (f64, f64, f64), minmax: (f64, f64), solvertype: SolverType) -> anyhow::Result<Self> {
        let inbus = RefBus::try_from(input_def).context(format!("PIDControllerの入力バスが不正です。"))?;
        let outbus = Bus::try_from(output_def).context(format!("PIDControllerの出力バスが不正です。"))?;

        if inbus.len() != 2 {
            return Err(anyhow!("PIDController: 入力信号の要素数は2個（1要素目：目標値、2要素目：現在値)で設定してください"))
        }

        if outbus.len() != 1 {
            return Err(anyhow!("PIDController: 出力信号の要素数は1個で設定してください"))
        }

        // Integrator用のBusを作る
        let mut integ_in = RefBus::try_from(vec![SigDef::new("integ_in", "-")]).unwrap();
        let integ_out = Bus::try_from(vec![SigDef::new("integ_out", "-")]).unwrap();
        
        let err_bus = Bus::try_from(vec![SigDef::new("error", "-")]).unwrap();
        integ_in.connect_to(&err_bus, &["error"], &["integ_in"]).unwrap();

        let integrator = Integrator::new(integ_in, integ_out, solvertype).unwrap();

        Ok(Self {
            integrator: integrator,
            u_old: 0.0,
            input_bus: inbus,
            output_bus: outbus,
            error_bus: err_bus,
            gain: gain,
            minmax: minmax,
        })
    }

    pub fn reset(&mut self) {
        self.integrator.reset(0.0);
    }
}

impl ModelCore for PIDController {
    fn initialize(&mut self, _sim_time: &SimTime) {
        self.integrator.reset(0.0);
    }

    fn finalize(&mut self) {
        // 処理なし
    }

    fn interface_in(&mut self) -> Option<&mut RefBus> {
        Some(&mut self.input_bus)
    }

    fn interface_out(&self) -> Option<&Bus> {
        Some(&self.output_bus)
    }

    fn nextstate(&mut self, sim_time: &SimTime) {
        
        let u = self.input_bus[0].val() - self.input_bus[1].val();
        self.error_bus[0].set_val(u); // 目標値 - 現在値      

        self.integrator.nextstate(sim_time); // 積分する
        
        let gain = self.gain;
        let integ = self.integrator.interface_out().unwrap()[0].val(); // 積分器の結果を取得
        let diff = (u - self.u_old) / sim_time.delta_t(); // 単純微分
        let o = gain.0 * u + gain.1 * integ + gain.2 * diff; // 出力計算
        
        self.output_bus[0].set_val(o.guard_minmax(self.minmax));

        self.u_old = u; // 前回値更新
        
    }
}