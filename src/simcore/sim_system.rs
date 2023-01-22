/// モデルを組み合わせて一つのシステムを構成する

use super::sim_model::model_core::ModelCore;
use super::sim_signal;
use sim_signal::bus::{Bus, RefBus};

pub struct SimSystem<'a> {
    delta_t: f64, // サンプルタイム
    sim_time: f64,
    models: Vec<Box<dyn ModelCore + 'a>>, // 個々のモデルを管理するベクタ　ひとまず、この配列順で実行する。将来的には、calc_orderなどの計算順を決める配列を用意する
                                          // Boxは参照しているのでstructの本体とライフタイムが一致する必要があるためライフタイムパラメータが必要
}

impl<'a> SimSystem<'a> {
    pub fn new(delta_t: f64) -> Self {
        Self {
            sim_time: 0.0,
            delta_t: delta_t,
            models: Vec::<Box<dyn ModelCore>>::new(),
        }
    }

    pub fn set_delta_t(&mut self, delta_t: f64) {
        self.delta_t = delta_t;
    }

    pub fn regist_model<T>(&mut self, model: T) 
        where T: ModelCore + 'a
    {
        self.models.push(Box::new(model));
    }

    pub fn nextstate(&mut self) {
        self.models.iter_mut().for_each(|mdl| mdl.nextstate(self.sim_time));
    }

    pub fn run(&mut self, sim_length: f64) {
        // 初期化処理
        self.initialize();

        // シミュレーション実行処理
        let step_num = (sim_length / self.delta_t) as i64 + 1;
        let printidx = (step_num - 1) / 10;
        let mut progress_cnt = 0; // 進捗カウンタ

        for idx in 0..=step_num {
            if idx % printidx == 0 {
                println!("processing now ... {}%\n", progress_cnt * 10);
                progress_cnt += 1;
            }

            self.nextstate();
        }

        // 終了処理
        self.finalize();
    }

    fn initialize(&mut self) {
        println!("Simulation Initializing Now ...\n");
        self.sim_time = 0.0;
        self.models.iter_mut().for_each(|mdl| mdl.initialize(self.delta_t));
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
            sim_time: 0.0,
            delta_t: 1.0, // デフォルト値
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
        let mut sys = SimSystem::new(0.5);
        
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

        sys.run(10.0);
        
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