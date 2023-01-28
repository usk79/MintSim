
use std::fmt;

mod buscore;
use buscore::BusCore;
use super::signal::{SigDef, Signal, RefSignal, SigTrait};

/// Busの定義
pub type Bus = BusCore<Signal>;
pub type RefBus = BusCore<RefSignal>;

use anyhow::{*};

extern crate nalgebra as na;
use na::{DMatrix};
impl Bus {
    pub fn import_matrix(&mut self, matrix: &DMatrix<f64>) { // DMatrixの値をBusに取り込む（要素数が同じであることが前提）
        matrix.iter().enumerate().for_each(|(i, m)| self[i].set_val(*m));
    }

    /// バス信号をすべてゼロリセットする
    pub fn zero_reset(&mut self) {
        self.iter_mut().for_each(|sig| sig.set_val(0.0));
    }

    /// バス信号をすべて指定値でリセットする
    pub fn set_all(&mut self, value: f64) {
        self.iter_mut().for_each(|sig| sig.set_val(value));
    }
    
}

/// Bus用の実装
impl TryFrom<Vec<SigDef>> for Bus {
    type Error = anyhow::Error;

    fn try_from(sigdef: Vec<SigDef>) -> anyhow::Result<Self> {

        let mut bus = Bus::new();

        for sig in sigdef.iter() {
            bus.push(Signal::new(0.0, sig.name(), sig.unit()))?
        }

        Ok(bus)
    }
}

impl fmt::Display for Bus {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        let buslen = self.len();
        let sigstr = self.iter().map(
            |sig| format!("  {}: {}[{}]", sig.name(), sig.val(), sig.unit())).collect::<Vec<String>>().join("\n");

        write!(f, "Bus: size = {}\nSignal List:\n{}\n", buslen, sigstr)
    }
}

/// RefBus用の実装
impl RefBus {
    pub fn connect_to<T:SigTrait> (&mut self, srcbus: &BusCore<T>, srclist: &[&str], dstlist: &[&str]) -> anyhow::Result<()> {
        if srclist.len() != dstlist.len() {
            return Err(anyhow!("接続元(srclist)と接続先の信号名リストのサイズが異なっています。"))
        }

        let mut notfoundlist_src: Vec<String> = Vec::new();
        let mut notfoundlist_dst: Vec<String> = Vec::new();

        for (idx, src_signame) in srclist.iter().enumerate() {
            let sig = srcbus.get_by_name(*src_signame);
            match sig {
                Some(s) => {
                    let dst_signame = dstlist[idx];
                    match self.get_by_name_mut(dst_signame) {
                        Some(sig) => sig.connect_to(s),
                        None => { 
                            notfoundlist_dst.push(format!("\"{}\"", dst_signame));
                            Ok(())
                        },
                    }?
                },
                None => notfoundlist_src.push(format!("\"{}\"", *src_signame))
            }
        }

        if notfoundlist_src.len() > 0 || notfoundlist_dst.len() > 0 {
            let errmsg = format!("RefBus内の信号を接続しようとしましたが、下記の信号が見つかりませんでした。
                                         \ndstlist: {}\nsrclist: {}\n",
                                         notfoundlist_dst.join(", "), 
                                         notfoundlist_src.join(", "));
            return Err(anyhow!(errmsg))
        }

        Ok(())
    }
    
    /// 指定した信号の接続を解除する
    pub fn disconnect(&mut self, signame: &str) -> anyhow::Result<()> {
        match self.get_by_name_mut(signame) {
            Some(s) => s.disconnect(),
            None => return Err(anyhow!("接続解除しようとしましたが、信号名{}が見つかりませんでした。", signame))
        }
        Ok(())
    }

    /// すべての信号の接続を解除する
    pub fn disconnect_all(&mut self) {
        self.iter_mut().for_each(|sig| sig.disconnect());
    }

    pub fn export_to_matrix(&self) -> DMatrix<f64> { // DMatrixの値をBusに取り込む（要素数が同じであることが前提）
        DMatrix::from_vec(self.len(), 1, self.to_vec_f64())
    }

}

impl TryFrom<Vec<SigDef>> for RefBus {
    type Error = anyhow::Error;

    fn try_from(sigdef: Vec<SigDef>) -> anyhow::Result<Self> {
        
        let mut bus = RefBus::new();

        for sig in sigdef.iter() {
            bus.push(RefSignal::new(sig.name(), sig.unit()))?
        }

        Ok(bus)
    }
}

impl fmt::Display for RefBus {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        let buslen = self.len();
        let sigstr = self.iter().map(
            |sig| format!("  {}", sig)).collect::<Vec<String>>().join("\n");

        write!(f, "RefBus: size = {}\nSignal List:\n{}\n", buslen, sigstr)
    }
}

#[cfg(test)]
mod sim_bus_test {
    use super::{*};

    #[test]
    fn bus_tryfrom() {
        let a = 
            Bus::try_from( vec![    
                SigDef::new("test1", "A"),
                SigDef::new("test2", "A"),
                SigDef::new("test3", "A"),
            ]).unwrap();
            
        assert_eq!(a[0].name(), "test1");
        assert_eq!(a[1].name(), "test2");
        assert_eq!(a[2].name(), "test3");

    }

    #[test]
    fn bus_fmt_display() {
        let mut a = 
        Bus::try_from( vec![    
            SigDef::new("test1", "A"),
            SigDef::new("test2", "A"),
            SigDef::new("test3", "A"),
        ]).unwrap();

        a[0].set_val(1.0);
        a[1].set_val(2.0);
        a[2].set_val(3.0);
        
        println!("{}", a);
        assert_eq!(format!("{}", a), format!("Bus: size = 3\nSignal List:\n  {}\n  {}\n  {}\n", a[0], a[1], a[2]));
    }

}

#[cfg(test)]
mod sim_refbus_test {
    
    use super::*;    

    #[test]
    fn refbus_tryfrom() {
        let a = 
            RefBus::try_from( vec![ 
                SigDef::new("test1", "A"),
                SigDef::new("test2", "A"),
                SigDef::new("test3", "A"),
            ]).unwrap();
            
        assert_eq!(a[0].name(), "test1");
        assert_eq!(a[1].name(), "test2");
        assert_eq!(a[2].name(), "test3");

    }

    #[test]
    fn refbus_fmt_display() {
        let a = 
        RefBus::try_from( vec![    
            SigDef::new("test1", "A"),
            SigDef::new("test2", "A"),
            SigDef::new("test3", "A"),
        ]).unwrap();
        
        println!("{}", a);
        assert_eq!(format!("{}", a), format!("RefBus: size = 3\nSignal List:\n  {}\n  {}\n  {}\n", a[0], a[1], a[2]));

    }

    #[test]
    fn refbus_connect() {
        let mut a = Bus::try_from( vec![    
                SigDef::new("bus1", "A"),
                SigDef::new("bus2", "A"),
                SigDef::new("bus3", "A"),
            ]).unwrap();

        a[0].set_val(1.0);
        a[1].set_val(2.0);
        a[2].set_val(3.0);
        
        let mut b = Bus::try_from( vec![    
            SigDef::new("bus4", "A"),
            SigDef::new("bus5", "A"),
            SigDef::new("bus6", "A"),
        ]).unwrap();
            
        b[0].set_val(11.0);
        b[1].set_val(12.0);
        b[2].set_val(13.0);
        
        let mut c = 
            RefBus::try_from( vec![
                SigDef::new("refbus1", "A"),
                SigDef::new("refbus2", "A"),
                SigDef::new("refbus3", "A"),
            ]).unwrap();
        
        c.connect_to(&a, &["bus1", "bus2"], &["refbus1", "refbus2"]).unwrap();
        c.connect_to(&b, &["bus6"], &["refbus3"]).unwrap();

        assert_eq!(c[0].val(), a[0].val());
        assert_eq!(c[1].val(), a[1].val());
        assert_eq!(c[2].val(), b[2].val());
        
        assert_eq!(c.get_by_name("refbus1").unwrap().val(), a.get_by_name("bus1").unwrap().val());
        assert_eq!(c.get_by_name("refbus2").unwrap().val(), a.get_by_name("bus2").unwrap().val());
        assert_eq!(c.get_by_name("refbus3").unwrap().val(), b.get_by_name("bus6").unwrap().val());
    }

    #[test]
    fn refbus_connect2() { // Refbusをさらに参照する場合
        let mut a = Bus::try_from( vec![    
            SigDef::new("bus1", "A"),
            SigDef::new("bus2", "A"),
            SigDef::new("bus3", "A"),
        ]).unwrap();

        a[0].set_val(1.0);
        a[1].set_val(2.0);
        a[2].set_val(3.0);
        
        let mut b = RefBus::try_from( vec![    
            SigDef::new("refbus1", "A"),
            SigDef::new("refbus2", "A"),
            SigDef::new("refbus3", "A"),
        ]).unwrap();

        b.connect_to(&a, &["bus3", "bus2", "bus1"], &["refbus1", "refbus2", "refbus3"]).unwrap();

        assert_eq!(b.get_by_name("refbus1").unwrap().val(), a.get_by_name("bus3").unwrap().val());
        assert_eq!(b.get_by_name("refbus2").unwrap().val(), a.get_by_name("bus2").unwrap().val());
        assert_eq!(b.get_by_name("refbus3").unwrap().val(), a.get_by_name("bus1").unwrap().val());
        
        let mut c = RefBus::try_from( vec![    
            SigDef::new("refbus1", "A"),
            SigDef::new("refbus2", "A"),
            SigDef::new("refbus3", "A"),
        ]).unwrap();

        c.connect_to(&b, &["refbus3", "refbus2", "refbus1"], &["refbus1", "refbus2", "refbus3"]).unwrap();

        assert_eq!(c.get_by_name("refbus1").unwrap().val(), b.get_by_name("refbus3").unwrap().val());
        assert_eq!(c.get_by_name("refbus2").unwrap().val(), b.get_by_name("refbus2").unwrap().val());
        assert_eq!(c.get_by_name("refbus3").unwrap().val(), b.get_by_name("refbus1").unwrap().val());

    }

    #[test]
    #[should_panic]
    fn refbus_connect_panic() {
        let a = Bus::try_from( vec![    
                SigDef::new("bus1", "A"),
                SigDef::new("bus2", "A"),
                SigDef::new("bus3", "A"),
            ]).unwrap();
        
        let b = Bus::try_from( vec![    
            SigDef::new("bus4", "A"),
            SigDef::new("bus5", "A"),
            SigDef::new("bus6", "A"),
        ]).unwrap();
                
        
        let mut c = 
            RefBus::try_from( vec![
                SigDef::new("refbus1", "A"),
                SigDef::new("refbus2", "A"),
                SigDef::new("refbus3", "A"),
            ]).unwrap();
        
        c.connect_to(&a, &["bus1", "bus2"], &["refbus1", "refbus2"]).unwrap();
        c.connect_to(&b, &["bus4"], &["refbus1"]).unwrap(); // 既に接続した信号に再接続はNG
    }

    #[test]
    #[should_panic]
    fn refbus_connect_notfound_panic() {
        let a = Bus::try_from( vec![    
                SigDef::new("bus1", "A"),
                SigDef::new("bus2", "A"),
                SigDef::new("bus3", "A"),
            ]).unwrap();
                
        
        let mut c = 
            RefBus::try_from( vec![
                SigDef::new("refbus1", "A"),
                SigDef::new("refbus2", "A"),
                SigDef::new("refbus3", "A"),
            ]).unwrap();
        
        c.connect_to(&a, &["bus5", "bus1"], &["refbus1", "refbus4"]).unwrap();
    }

    #[test]
    fn refbus_disconnect_all() {
        let mut a = Bus::try_from( vec![    
            SigDef::new("bus1", "A"),
            SigDef::new("bus2", "A"),
            SigDef::new("bus3", "A"),
        ]).unwrap();
        
        let mut c = 
            RefBus::try_from( vec![
                SigDef::new("refbus1", "A"),
                SigDef::new("refbus2", "A"),
                SigDef::new("refbus3", "A"),
            ]).unwrap();
        
        a[0].set_val(1.0);
        a[1].set_val(2.0);
        a[2].set_val(3.0);

        c.connect_to(&a, &["bus1", "bus2", "bus3"], &["refbus1", "refbus2", "refbus3"]).unwrap();
        assert_eq!(a[0].val(), c[0].val());
        assert_eq!(a[1].val(), c[1].val());
        assert_eq!(a[2].val(), c[2].val());

        c.disconnect_all();

        c.connect_to(&a, &["bus3", "bus2", "bus1"], &["refbus1", "refbus2", "refbus3"]).unwrap();
        assert_eq!(a[2].val(), c[0].val());
        assert_eq!(a[1].val(), c[1].val());
        assert_eq!(a[0].val(), c[2].val());

    }

    #[test]
    fn refbus_disconnect() {
        let mut a = Bus::try_from( vec![    
            SigDef::new("bus1", "A"),
            SigDef::new("bus2", "A"),
            SigDef::new("bus3", "A"),
        ]).unwrap();
        
        let mut c = 
            RefBus::try_from( vec![
                SigDef::new("refbus1", "A"),
                SigDef::new("refbus2", "A"),
                SigDef::new("refbus3", "A"),
            ]).unwrap();
        
        a[0].set_val(1.0);
        a[1].set_val(2.0);
        a[2].set_val(3.0);

        c.connect_to(&a, &["bus1", "bus2", "bus3"], &["refbus1", "refbus2", "refbus3"]).unwrap();
        assert_eq!(a[0].val(), c[0].val());
        assert_eq!(a[1].val(), c[1].val());
        assert_eq!(a[2].val(), c[2].val());

        c.disconnect("refbus1").unwrap();
        c.disconnect("refbus3").unwrap();

        c.connect_to(&a, &["bus1", "bus3"], &["refbus3", "refbus1"]).unwrap();
        assert_eq!(a[2].val(), c[0].val());
        assert_eq!(a[1].val(), c[1].val());
        assert_eq!(a[0].val(), c[2].val());

    }
    
    #[test]
    #[should_panic]
    fn refbus_disconnect_panic() {
        let mut a = Bus::try_from( vec![    
            SigDef::new("bus1", "A"),
            SigDef::new("bus2", "A"),
            SigDef::new("bus3", "A"),
        ]).unwrap();
        
        let mut c = 
            RefBus::try_from( vec![
                SigDef::new("refbus1", "A"),
                SigDef::new("refbus2", "A"),
                SigDef::new("refbus3", "A"),
            ]).unwrap();
        
        a[0].set_val(1.0);
        a[1].set_val(2.0);
        a[2].set_val(3.0);

        c.connect_to(&a, &["bus1", "bus2", "bus3"], &["refbus1", "refbus2", "refbus3"]).unwrap();
        assert_eq!(a[0].val(), c[0].val());
        assert_eq!(a[1].val(), c[1].val());
        assert_eq!(a[2].val(), c[2].val());

        c.disconnect("refbus4").unwrap();
        c.disconnect("refbus3").unwrap();

        c.connect_to(&a, &["bus1", "bus3"], &["refbus3", "refbus1"]).unwrap();
        assert_eq!(a[2].val(), c[0].val());
        assert_eq!(a[1].val(), c[1].val());
        assert_eq!(a[0].val(), c[2].val());

    }

    #[test]
    fn import_matrix() {
        let m = DMatrix::from_vec(3, 1, vec![1.0, 2.0, 3.0]);
        let mut a = Bus::try_from( vec![    
            SigDef::new("bus1", "A"),
            SigDef::new("bus2", "A"),
            SigDef::new("bus3", "A"),
        ]).unwrap();
        
        a.import_matrix(&m);
        assert_eq!(a[0].val(), 1.0);
        assert_eq!(a[1].val(), 2.0);
        assert_eq!(a[2].val(), 3.0);

    }

    #[test]
    fn export_matrix() {
        let mut a = Bus::try_from( vec![    
            SigDef::new("bus1", "A"),
            SigDef::new("bus2", "A"),
            SigDef::new("bus3", "A"),
        ]).unwrap();

        a[0].set_val(1.0);
        a[1].set_val(2.0);
        a[2].set_val(3.0);
        
        let mut b = RefBus::try_from(vec![
            SigDef::new("refbus1", "A"),
            SigDef::new("refbus2", "A"),
            SigDef::new("refbus3", "A"),
        ]).unwrap();

        b.connect_to(&a, &["bus1", "bus2", "bus3"], &["refbus1", "refbus2", "refbus3"]).unwrap();
        
        let mat = b.export_to_matrix();
        
        assert_eq!(mat[0], 1.0);
        assert_eq!(mat[1], 2.0);
        assert_eq!(mat[2], 3.0);
        
    }
}
