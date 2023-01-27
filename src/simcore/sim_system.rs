/// モデルを組み合わせて一つのシステムを構成する

use super::sim_model::model_core::{ModelCore, DEFAULT_DELTA_T};
use super::sim_signal;
use sim_signal::bus::{Bus, RefBus};

/// SimTime
/// シミュレーションの時間を管理
#[derive(Clone, Copy, Debug)]
pub struct SimTime {
    time: f64,       // シミュレーション時刻
    step: u32,       // ステップ番号
    delta_t: f64,    // シミュレーション刻み幅
    start_time: f64, // 開始時刻
    end_time: f64,   // 終了時刻
}

impl SimTime {
    pub fn new(start_time: f64, end_time: f64, delta_t: f64) -> Self {
        Self {
            time: start_time,
            step: 0,
            delta_t: delta_t,
            start_time: start_time,
            end_time: end_time,
        }
    }

    /// シミュレーション時間をリセットする
    pub fn reset(&mut self) {
        self.step = 0;
        self.time = self.start_time;
    }

    /// シミュレーション刻み幅Δtを変更する
    pub fn change_delta_t(&mut self, delta_t: f64) {
        self.delta_t = delta_t;
    }
    /// シミュレーション時刻を1ステップ進める
    pub fn nextstate(&mut self) {
        self.time = self.time + self.delta_t;
        self.step += 1;
    }
    /// 現在時刻を取得する
    pub fn time(&self) -> f64 {
        self.time
    }
    /// シミュレーション刻み幅Δtを取得する
    pub fn delta_t(&self) -> f64 {
        self.delta_t
    }
    /// シミュレーションのステップ数を取得する
    pub fn step_num(&self) -> u32 {
        ((self.end_time - self.start_time) / self.delta_t) as u32
    }
}

/// SimTimeのイテレータ実装
impl Iterator for SimTime {
    type Item = (u32, f64); // (step, time)

    fn next(&mut self) -> Option<Self::Item> {
        if self.time < self.end_time {
            self.nextstate();
            Some((self.step, self.time))
        } else {
            None
        }
    }
}

/// SimSystem
/// モデル同士の接続とシミュレーションの実行を司る
pub struct SimSystem<'a> {
    sim_time: SimTime,
    models: Vec<Box<dyn ModelCore + 'a>>, // 個々のモデルを管理するベクタ　ひとまず、この配列順で実行する。将来的には、calc_orderなどの計算順を決める配列を用意する
                                          // Boxは参照しているのでstructの本体とライフタイムが一致する必要があるためライフタイムパラメータが必要
}

impl<'a> SimSystem<'a> {
    pub fn new(start_time: f64, end_time: f64, delta_t: f64) -> Self {
        Self {
            sim_time: SimTime::new(start_time, end_time, delta_t),
            models: Vec::<Box<dyn ModelCore>>::new(),
        }
    }

    pub fn set_sim_time(&mut self, sim_time: SimTime) {
        self.sim_time = sim_time;
    }

    pub fn sim_time(&mut self) -> &mut SimTime {
        &mut self.sim_time
    }

    pub fn regist_model<T>(&mut self, model: T) 
        where T: ModelCore + 'a
    {
        self.models.push(Box::new(model));
    }

    pub fn nextstate(&mut self) {
        self.models.iter_mut().for_each(|mdl| mdl.nextstate(&self.sim_time));
    }

    pub fn run(&mut self) {
        // 初期化処理
        self.initialize();

        // シミュレーション実行処理
        let print_interval = self.sim_time.step_num() / 10;
        let mut print_cnt = 0;
        let mut progress_cnt = 0; // 進捗カウンタ
        
        for _s in self.sim_time.into_iter() {
            print_cnt += 1;
            if print_cnt >= print_interval {
                progress_cnt += 1;
                print_cnt = 0;
                println!("processing now ... {}%)\n", progress_cnt * 10);
            }
            self.nextstate();
        }
        
        // 終了処理
        self.finalize();
    }

    fn initialize(&mut self) {
        println!("Simulation Initializing Now ...\n");
        self.sim_time.reset();
        self.models.iter_mut().for_each(|mdl| mdl.initialize());
    }

    fn finalize(&mut self) {
        println!("Simulation Finalizing Now ...\n");
        self.models.iter_mut().for_each(|mdl| mdl.finalize());
    }
}

impl<'a> From<Vec<Box<dyn ModelCore>>> for SimSystem<'a>
{
    fn from(mdl_list: Vec<Box<dyn ModelCore>>) -> Self
    {
        Self {
            sim_time: SimTime { time: 0.0, step: 0, delta_t: DEFAULT_DELTA_T, start_time: 0.0, end_time: 1.0 },
            models: mdl_list,
        }
    }
}

#[cfg(test)]
mod sim_bus_test {
    use super::{*};
    use super::super::sim_model::test_models::{*};
    use super::super::sim_signal::signal::{*};

    #[test]
    fn system_regist_test() {
        let mut sys = SimSystem::new(0.0, 10.0, 0.01);
        
        let mut mdl1 = TestModel::new();
        let mut mdl2 = TestModel::new();

        let mut datbus = Bus::try_from(vec![
            SigDef::new("data1", "kW"),
            SigDef::new("data2", "t"),
        ]).unwrap();


        datbus[0].set_val(1.0);
        datbus[1].set_val(2.0);
        
        let inbus = mdl1.interface_in().unwrap();
        inbus.connect_to(&datbus, &["data1", "data2"], &["test_in1", "test_in2"]).unwrap();
        
        mdl2.interface_in().unwrap().connect_to(mdl1.interface_out().unwrap(), 
                        &["test_out1", "test_out2"], 
                        &["test_in1", "test_in2"]).unwrap();

        sys.regist_model(mdl1);
        sys.regist_model(mdl2);

        sys.nextstate();
        
        let outbus = sys.models[0].interface_out().unwrap();
        assert_eq!(outbus[0].val(), 2.0);
        assert_eq!(outbus[1].val(), 1.0);

        let outbus = sys.models[1].interface_out().unwrap();
        assert_eq!(outbus[0].val(), 2.0);
        assert_eq!(outbus[1].val(), 2.0);

        sys.run();
        
    }

    #[test]
    fn system_from_test() { 
        /*
        let mut mdl1 = TestModel::new();
        let mut mdl2 = TestModel2::new();

        let sys = vec![Box::new(TestModel::new()), Box::new(TestModel2::new())].into();
        */
    }
}