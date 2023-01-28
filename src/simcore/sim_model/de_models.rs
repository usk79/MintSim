/// # DEモデル
/// DEモデルには、下記のモデルを実装する

/// - 状態空間モデル
/// - 微分方程式モデル
/// - 伝達関数モデル
/// - 積分器モデル
use anyhow::anyhow;

extern crate nalgebra as na;
use na::{DMatrix};

use super::model_core::{ModelCore};

use super::super::sim_signal;
use sim_signal::signal::{SigDef, SigTrait};

use sim_signal::bus::{Bus, RefBus};

use super::super::sim_system;
use sim_system::SimTime;

/// DEModelトレイト
pub trait DEModel: ModelCore {
    fn derivative_func(&self, x: &DMatrix<f64>) -> DMatrix<f64>; // 導関数を定義する
    
    fn set_state(&mut self, newstate: DMatrix<f64>);
    
    fn get_state(&self) -> &DMatrix<f64>;

    fn euler_method(&mut self, delta_t: f64) {
        let state = self.get_state();
        let newstate = state + self.derivative_func(state) * delta_t;
        self.set_state(newstate);
    }

    fn rungekutta_method(&mut self, delta_t: f64) {
        let state = self.get_state();
        let d1 = self.derivative_func(state) * delta_t;
        let d2 = self.derivative_func(&(state + &d1 / 2.0)) * delta_t;
        let d3 = self.derivative_func(&(state + &d2 / 2.0)) * delta_t;
        let d4 = self.derivative_func(&(state + &d3)) * delta_t;
        let newstate = state + (d1 + 2.0 * d2 + 2.0 * d3 + d4) / 6.0;
        self.set_state(newstate);
    }
}

/// 常微分方程式のソルバ
#[derive(Debug, Clone)]
pub enum SolverType {
    Euler,
    RungeKutta,
}

/// 状態空間モデル
#[derive(Debug, Clone)]
pub struct SpaceStateModel {
    mtrx_a: DMatrix<f64>,    // 状態遷移行列A
    mtrx_b: DMatrix<f64>,    // 入力行列B
    mtrx_c: DMatrix<f64>,    // 観測行列C
    mtrx_d: DMatrix<f64>,    // 入力行列D
    state_dim: usize,        // 状態次数
    input_dim: usize,        // 入力次数
    output_dim: usize,       // 出力次数
    x: DMatrix<f64>,         // 状態ベクトル
    solver: SolverType,      // ソルバータイプ
    input_bus: RefBus,
    output_bus: Bus,
}

impl SpaceStateModel {
    pub fn new(inbus: RefBus, outbus: Bus, sdim: usize, solvertype: SolverType) -> anyhow::Result<Self> {
        let idim = inbus.len();
        let odim = outbus.len();
        if sdim <= 0 || idim <= 0 || odim <= 0 {
            return Err(anyhow!("状態, 入力, 出力の次数は自然数である必要があります。"));
        }

        Ok(SpaceStateModel {
            mtrx_a: DMatrix::from_element(sdim, sdim, 0.0),
            mtrx_b: DMatrix::from_element(sdim, idim, 0.0),
            mtrx_c: DMatrix::from_element(odim, sdim, 0.0),
            mtrx_d: DMatrix::from_element(odim, idim, 0.0),
            x: DMatrix::from_element(sdim, 1, 0.0),
            state_dim: sdim,
            input_dim: idim,
            output_dim: odim,
            solver: solvertype,
            input_bus: inbus, 
            output_bus: outbus,
        })
    }

    pub fn init_state(&mut self, init_state: &[f64]) -> anyhow::Result<()> {
        self.set_x(init_state)?;
        Ok(())
    }

    pub fn set_x(&mut self, x: &[f64]) -> anyhow::Result<()> {
        if x.len() != self.state_dim {
            return Err(anyhow!("状態ベクトルの次数が違います。"))
        }
        x.iter().enumerate().for_each(|(i, e)| self.x[i] = *e);

        Ok(())
    }

    pub fn set_mtrx_a(&mut self, mtrx_a: &[f64]) -> anyhow::Result<()> {

        if mtrx_a.len() != self.state_dim * self.state_dim {
            return Err(anyhow!("A行列のサイズが違います。"));
        }
        mtrx_a.iter().enumerate().for_each(|(i, e)| self.mtrx_a[(i / self.state_dim, i % self.state_dim)] = *e);

        Ok(())
    }

    pub fn set_mtrx_b(&mut self, mtrx_b: &[f64]) -> anyhow::Result<()> {
        
        if mtrx_b.len() != self.state_dim * self.input_dim {
            return Err(anyhow!("B行列のサイズが違います。"));
        }

        mtrx_b.iter().enumerate().for_each(|(i, e)| self.mtrx_b[(i / self.input_dim, i % self.input_dim)] = *e);

        Ok(())
    }

    pub fn set_mtrx_c(&mut self, mtrx_c: &[f64]) -> anyhow::Result<()> {

        if mtrx_c.len() != self.output_dim * self.state_dim {
            return Err(anyhow!("C行列のサイズが違います。"));
        }
        mtrx_c.iter().enumerate().for_each(|(i, e)| self.mtrx_c[(i / self.state_dim, i % self.state_dim)] = *e);

        Ok(())
    }

    pub fn set_mtrx_d(&mut self, mtrx_d: &[f64]) -> anyhow::Result<()> {

        if mtrx_d.len() != self.output_dim * self.input_dim {
            return Err(anyhow!("D行列のサイズが違います。"));
        }
        mtrx_d.iter().enumerate().for_each(|(i, e)| self.mtrx_d[(i / self.input_dim, i % self.input_dim)] = *e);

        Ok(())
    }

    pub fn get_observation(&self) -> DMatrix<f64> {
        let u = self.input_bus.export_to_matrix();
        &self.mtrx_c * &self.x + &self.mtrx_d * u
    }

}

impl ModelCore for SpaceStateModel {
    fn initialize(&mut self) {
        
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
        let delta_t = sim_time.delta_t();
        match self.solver { 
            SolverType::Euler => self.euler_method(delta_t),
            SolverType::RungeKutta => self.rungekutta_method(delta_t),
        }

        let obs = self.get_observation();

        self.output_bus.import_matrix(&obs);
    }
}

impl DEModel for SpaceStateModel {
    fn derivative_func(&self, x: &DMatrix<f64>) -> DMatrix<f64> {
        let u = self.input_bus.export_to_matrix();
        &self.mtrx_a * x + &self.mtrx_b * &u
    }

    fn set_state(&mut self, newstate: DMatrix<f64>) {
        self.x = newstate; 
    }

    fn get_state(&self) -> &DMatrix<f64> {
        &self.x
    }
}