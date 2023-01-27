use super::model_core::{ModelCore};

use super::super::sim_signal;
use sim_signal::signal::{SigDef, SigTrait};

use sim_signal::bus::{Bus, RefBus};

use super::super::sim_system;
use sim_system::SimTime;

pub struct TestModel {
    inbus: RefBus,
    outbus: Bus,
    state: f64,
}

impl TestModel {
    pub fn new() -> Self {
        let inbus = RefBus::try_from(vec![
            SigDef::new("test_in1", "A"),
            SigDef::new("test_in2", "V"),
        ]).unwrap();

        let mut outbus = Bus::try_from(vec![
            SigDef::new("test_out1", "kW"),
            SigDef::new("test_out2", "t"),
        ]).unwrap();

        outbus.iter_mut().for_each(|x| x.set_val(0.0)); // 初期化

        Self {
            inbus: inbus,
            outbus: outbus,
            state: 0.0,
        }
    }
}

impl ModelCore for TestModel {
    fn initialize(&mut self) {
        self.state = 0.0;
    }

    fn finalize(&mut self) {}

    fn nextstate(&mut self, _sim_time: &SimTime) {
        let in1 = self.inbus[0].val();
        let in2 = self.inbus[1].val();

        
        self.state = self.state + in1;
        
        self.outbus[0].set_val(in1 * in2);
        self.outbus[1].set_val(self.state);        
    }

    fn interface_in(&mut self) -> Option<&mut RefBus> {
        Some(&mut self.inbus)
    }

    fn interface_out(&self) -> Option<&Bus> {
        Some(&self.outbus)
    }
}

pub struct TestModel2 {
    inbus: RefBus,
    outbus: Bus,
    state: f64,
}

impl TestModel2 {
    pub fn new() -> Self {
        let inbus = RefBus::try_from(vec![
            SigDef::new("test_in1", "A"),
            SigDef::new("test_in2", "V"),
        ]).unwrap();

        let mut outbus = Bus::try_from(vec![
            SigDef::new("test_out1", "kW"),
            SigDef::new("test_out2", "t"),
        ]).unwrap();

        outbus.iter_mut().for_each(|x| x.set_val(0.0)); // 初期化

        Self {
            inbus: inbus,
            outbus: outbus,
            state: 0.0,
        }
    }
}

impl ModelCore for TestModel2 {
    fn initialize(&mut self) {
        self.state = 0.0;
    }
    fn finalize(&mut self) {}

    fn nextstate(&mut self, _sim_time: &SimTime) {
        let in1 = self.inbus[0].val();
        let in2 = self.inbus[1].val();

        
        self.state = self.state + in1;
        
        self.outbus[0].set_val(in1 * in2);
        self.outbus[1].set_val(self.state);        
    }

    fn interface_in(&mut self) -> Option<&mut RefBus> {
        Some(&mut self.inbus)
    }

    fn interface_out(&self) -> Option<&Bus> {
        Some(&self.outbus)
    }
}