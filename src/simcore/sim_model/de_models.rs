/// # DEモデル
/// DEモデルには、下記のモデルを実装する

/// - 状態空間モデル
/// - 微分方程式モデル
/// - 伝達関数モデル
/// - 積分器モデル

use std::fmt;
use anyhow::{anyhow, Context};

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
    init_x: DMatrix<f64>,    // 初期状態
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
            init_x: DMatrix::from_element(sdim, 1, 0.0), // 初期状態はデフォルトは0ベクトル
            state_dim: sdim,
            input_dim: idim,
            output_dim: odim,
            solver: solvertype,
            input_bus: inbus, 
            output_bus: outbus,
        })
    }

    pub fn set_init_state(&mut self, init_state: &[f64]) -> anyhow::Result<()> {
        if init_state.len() != self.state_dim {
            return Err(anyhow!("状態ベクトルの次数が違います。"))
        }
        init_state.iter().enumerate().for_each(|(i, e)| self.init_x[i] = *e);
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
        self.x = self.init_x.clone();
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

impl fmt::Display for SpaceStateModel {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        // ここに状態モデルを見やすく表示するための処理を記述する
        let mut str_a = String::from(format!("Matrix A ({0} x {0}): \n", self.state_dim));
        for r in 0..self.state_dim {
            str_a = str_a + &"|";
            for c in 0..self.state_dim {
                str_a = str_a + &format!("{:>15.5} ", self.mtrx_a[(r, c)]);
            }
            str_a = str_a + &format!("|\n")
        }

        let mut str_b = String::from(format!("Matrix B ({} x {}): \n", self.state_dim, self.input_dim));
        for r in 0..self.state_dim {
            str_b = str_b + &"|";
            for c in 0..self.input_dim {
                str_b = str_b + &format!("{:>15.5} ", self.mtrx_b[(r, c)]);
            }
            str_b = str_b + &format!("|\n")
        }

        let mut str_c = String::from(format!("Matrix C ({} x {}): \n", self.output_dim, self.state_dim));
        for r in 0..self.output_dim {
            str_c = str_c + &"|";
            for c in 0..self.state_dim {
                str_c = str_c + &format!("{:>15.5} ", self.mtrx_c[(r, c)]);
            }
            str_c = str_c + &format!("|\n")
        }

        let mut str_d = String::from(format!("Matrix D ({} x {}): \n", self.output_dim, self.input_dim));
        for r in 0..self.output_dim {
            str_d = str_d + &"|";
            for c in 0..self.input_dim {
                str_d = str_d + &format!("{:>15.5} ", self.mtrx_d[(r, c)]);
            }
            str_d = str_d + &format!("|\n")
        }

        
        write!(f, "{}\n{}\n{}\n{}\n", str_a, str_b, str_c, str_d)
    }
}

// 伝達関数から状態空間モデルを生成する
pub fn crate_ssm_from_tf<'a> (num: &'a [f64], den: &'a [f64], inbus: RefBus, outbus: Bus, solvertype: SolverType) -> anyhow::Result<SpaceStateModel> {
    let sdim = den.len() - 1;
    let idim = 1;
    let odim = 1;

    if sdim < 1 {
        return Err(anyhow!("状態ベクトルの次数が0になりました。"))
    }
    if num.len() > den.len() {
        return Err(anyhow!("プロパーな伝達関数ではありません。"))
    }
    if inbus.len() != 1 || outbus.len() != 1{
        return  Err(anyhow!("伝達関数における入出力バスの次元は1である必要があります。"));
     }

    let mut model = SpaceStateModel::new(inbus, outbus, sdim, solvertype)?;
    let an = den[0];

    // A行列の作成
    let mut mtrx_a = vec![0.0; sdim * sdim];
    
    for r in 0..sdim {
        for c in 0..sdim {
            if c == sdim - 1 {
                mtrx_a[r * sdim + c] = -den[sdim - r] / an;
            }
            else {
                mtrx_a[r * sdim + c] = 0.0;
            }
        }

        if r > 0 {
            mtrx_a[r * sdim + r - 1] = 1.0;
        }
    }

    model.set_mtrx_a(&mtrx_a).context("failed at from_tf()")?;

    // B行列の作成
    let mut mtrx_b = vec![0.0; sdim * idim];
    let bn = 
        if num.len() - 1 == sdim {
            num[0]
        } else {
            0.0
        };
    
    for r in 0..sdim {
        if r < num.len() {
            mtrx_b[r] = (num[num.len() - r - 1] - den[sdim - r] * bn ) / an;
        } else {
            mtrx_b[r] = (-den[sdim - r] * bn ) / an;
        }

    }
    model.set_mtrx_b(&mtrx_b).context("failed at from_tf()")?;

    // C行列の作成
    let mut mtrx_c = vec![0.0; sdim * odim];
    mtrx_c[sdim - 1] = 1.0;
    model.set_mtrx_c(&mtrx_c).context("failed at from_tf()")?;

    // D行列の作成
    let mtrx_d = vec![bn; odim * idim];
    model.set_mtrx_d(&mtrx_d).context("failed at from_tf()")?;

    Ok(model)
}

#[cfg(test)]
mod simmodel_test {
    use super::*;
    // cargo test -- --test-threads=1　シングルスレッドで実行したいとき
    /*fn print_typename<T>(_: T) {
        println!("{}", std::any::type_name::<T>());
    }*/
    #[test] // StateSpaceModelのセット時のテスト 
    fn ssm_settest() {
        let input = RefBus::try_from(vec![SigDef::new("i1", "Nm")]).unwrap();
        let output = Bus::try_from(vec![SigDef::new("o1", "rpm")]).unwrap();
        let mut model = SpaceStateModel::new(input, output, 2, SolverType::Euler).unwrap();

        let mtrx_a = [1.0, 1.0, 1.0, 1.0];
        model.set_mtrx_a(&mtrx_a).unwrap();
        assert_eq!(model.mtrx_a, DMatrix::from_row_slice(2, 2, &mtrx_a));

        let mtrx_b = [1.0, 1.0];
        model.set_mtrx_b(&mtrx_b).unwrap();
        assert_eq!(model.mtrx_b, DMatrix::from_row_slice(2, 1, &mtrx_b));

        let mtrx_c = [1.0, 1.0];
        model.set_mtrx_c(&mtrx_c).unwrap();
        assert_eq!(model.mtrx_c, DMatrix::from_row_slice(1, 2, &mtrx_c));

        let init_state = [1.0, 2.0];
        model.set_init_state(&init_state).unwrap();
        assert_eq!(model.init_x, DMatrix::from_row_slice(2, 1, &init_state));

        let x = [1.0, 2.0];
        model.set_x(&x).unwrap();
        assert_eq!(model.x, DMatrix::from_row_slice(2, 1, &init_state));

        let mtrx_d = [1.0];
        model.set_mtrx_d(&mtrx_d).unwrap();
        assert_eq!(model.mtrx_d, DMatrix::from_row_slice(1, 1, &mtrx_d));

        println!("model : {}\n", model);
    }

    #[test]
    fn ssm_interfacetest() {

        let mut input_bus = RefBus::try_from(vec![
            SigDef::new("i1", "Nm"),
        ]).unwrap();

        let output_bus = Bus::try_from(vec![
            SigDef::new("o1", "rpm"),
            SigDef::new("o2", "Nm"),
        ]).unwrap();
        
        let mut databus = Bus::try_from(vec![
            SigDef::new("d1", "Nm")
        ]).unwrap();

        databus[0].set_val(1.0);

        input_bus.connect_to(&databus, &["d1"], &["i1"]).unwrap();

        let mut model = SpaceStateModel::new(input_bus, output_bus, 2, SolverType::Euler).unwrap();
        model.set_mtrx_a(&[1.0, 0.0, 0.0, 1.0]).unwrap();
        model.set_mtrx_b(&[1.0, 2.0]).unwrap();
        model.set_mtrx_c(&[1.0, 0.0, 0.0, 1.0]).unwrap();

        
        let simtime = SimTime::new(0.0, 1.0, 1.0);
        model.nextstate(&simtime);  

        let output = model.interface_out().unwrap(); // 出力のテスト
        assert_eq!(output.get_by_name("o1").unwrap().val(), 1.0);
        assert_eq!(output.get_by_name("o2").unwrap().val(), 2.0);
        
        println!("output => \n{}", output);
    }

}