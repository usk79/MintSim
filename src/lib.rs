//extern crate anyhow;


pub mod simcore;

#[cfg(test)]
mod tests {
    
    use crate::simcore::sim_model::{test_models::TestModel, model_core::ModelCore};
    use crate::simcore::sim_signal::{bus, signal};
    use crate::simcore::sim_system::SimSystem;
    use bus::{*};
    use signal::{*};
    

    //use super::simcore::sim_signal::bus::{Bus};
    //use super::simcore::sim_signal::signal::{Signal};

    #[test]
    fn model_unittest() {
        /*
        let mut mdl = TestModel::new();
        let mut datbus = Bus::try_from(vec![
            SigDef::new("data1", "kW"),
            SigDef::new("data2", "t"),
        ]).unwrap();

        datbus[0].set_val(1.0);
        datbus[1].set_val(2.0);

        let inbus = mdl.interface_in().unwrap();
        inbus.connect_to(&datbus, &["data1", "data2"], &["test_in1", "test_in2"]).unwrap();

        mdl.nextstate(1.0);

        let outbus = mdl.interface_out().unwrap();
        
        assert_eq!(outbus[0].val(), 2.0);
        assert_eq!(outbus[1].val(), 1.0);
        
        datbus[0].set_val(2.0);
        datbus[1].set_val(3.0);

        mdl.nextstate(1.0);
        
        let outbus = mdl.interface_out().unwrap();
        
        assert_eq!(outbus[0].val(), 6.0);
        assert_eq!(outbus[1].val(), 3.0);
        */
    }

    
}
