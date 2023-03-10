
use std::fmt;
use anyhow::{*};

/// 信号名と単位だけを設定する用のタプル
#[derive(Debug, Clone, PartialEq)]
pub struct SigDef(String, String); // (信号名, 単位)

impl SigDef {
    pub fn new(name: impl Into<String>, unit: impl Into<String>) -> Self {
        Self(name.into(), unit.into())
    }
    
    pub fn name(&self) -> &str {
        &self.0
    }

    pub fn unit(&self) -> &str {
        &self.1
    }
}

impl fmt::Display for SigDef {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "{}[{}]", self.0, self.1)
    }
}

/// SigTrait
pub trait SigTrait {
    /// 信号名を取得する
    fn name(&self) -> String; // Rcを使っているためBorrowするときに一時変数が発生することとなり、コピーを作らないと渡せないため&str ではなく Stringにしている

    /// 単位を取得する
    fn unit(&self) -> String;

    /// 値を取得する
    fn val(&self) -> f64;
    
    fn sig(&self) -> &Rc<RefCell<SigCore>>;
}

// SigCore構造体
/// シミュレーションで使用する単一の信号表現
/// 値、信号名、信号の単位のデータを保存する。
use std::rc::Rc;
use std::cell::{RefCell};
#[derive(Debug, Clone, PartialEq)]
pub struct SigCore {
    value: f64,   
    sigdef: SigDef,
}

impl SigCore {
    fn new(initvalue: f64, name: impl Into<String>, unit: impl Into<String>) -> Self {
        Self {
            value: initvalue,
            sigdef: SigDef::new(name.into(), unit.into()),
        }
    }

    fn name(&self) -> String {
        String::from(self.sigdef.name())
    }

    fn unit(&self) -> String {
        String::from(self.sigdef.unit())
    }

    fn val(&self) -> f64 {
        self.value
    }
}

impl fmt::Display for SigCore {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "{}: {}[{}]", self.sigdef.name(), self.value, self.sigdef.unit())
    }
}


#[derive(Debug, Clone, PartialEq)]
pub struct Signal {
    sig: Rc<RefCell<SigCore>>,   
}

impl Signal {
    pub fn new(initvalue: f64, name: impl Into<String>, unit: impl Into<String>) -> Self {
        Self {
            sig: Rc::new(RefCell::new(SigCore::new(initvalue, name, unit))),
        }
    }

    pub fn set_val(&mut self, val: f64) {
        self.sig.borrow_mut().value = val;
    }
}

impl SigTrait for Signal {
    fn name(&self) -> String {
        self.sig.borrow().name()
    }

    fn unit(&self) -> String {
        self.sig.borrow().unit()
    }

    fn val(&self) -> f64 {
        self.sig.borrow().val()
    }

    fn sig(&self) -> &Rc<RefCell<SigCore>> {
        &self.sig
    }
} 

impl fmt::Display for Signal {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        self.sig.borrow().fmt(f)
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct RefSignal {
    sig: Option<Rc<RefCell<SigCore>>>,
    sigdef: SigDef, // 信号名が変わってもいいためsigdefを定義
}

impl RefSignal {
    pub fn new(name: impl Into<String>, unit: impl Into<String>) -> Self {
        let sigdef = SigDef::new(name, unit);
        Self {
            sig: None,
            sigdef: sigdef,
        }
    }

    pub fn connect_to<T:SigTrait>(&mut self, signal: &T) -> anyhow::Result<()> { 
        if self.sig.is_some() {
            return Err(anyhow!("既に信号が接続されています。\n接続元の信号名:{}\n接続先の信号名:{}\n", self.sigdef.name(), signal.name()));
        }

        self.sig = Some(Rc::clone(&signal.sig()));

        Ok(())
    }

    pub fn is_connected(&self) -> bool {
        self.sig.is_some()
    }

    pub fn disconnect(&mut self) {
        self.sig = None;
    }
}

impl SigTrait for RefSignal {
    fn name(&self) -> String {
       String::from(self.sigdef.name())
    }

    fn unit(&self) -> String {
        String::from(self.sigdef.unit())
    }

    fn val(&self) -> f64{
        match &self.sig {
            Some(sig) => sig.borrow().val(),
            None => panic!("RefSignalの参照先が設定されていません。信号名:{}", self.name())
        }
    }

    fn sig(&self) -> &Rc<RefCell<SigCore>> {
        &self.sig.as_ref().unwrap()
    }
}

impl fmt::Display for RefSignal {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match &self.sig {
            Some(sig) => {
                write!(f, "{}: {} [{}] Referrer: {}[{}]", 
                    self.sigdef.name(), self.val(), self.sigdef.unit(),
                    sig.borrow().name(), sig.borrow().unit())
                
            }
            None => write!(f, "{} [{}] Referrer: Not Connected!", 
                    self.sigdef.name(), self.sigdef.unit())
        }
    }
}


#[cfg(test)]
mod sim_signals_test {
    use super::*;

    #[test]
    fn signaltest() {
        let sig = Signal::new(2.0, "a", "A");

        assert_eq!(sig.val(), 2.0);
        assert_eq!(sig.name(), "a");
        assert_eq!(sig.unit(), "A");
    }

    #[test]
    fn printtest() {
        // 表示する時は cargo test -- --nocapture にて実行
        let a = Signal::new(1.1, "motor_current", "A");
        let mut b = RefSignal::new("motor2", "A");
        
        let s = format!("{}", a);
        assert_eq!(s, "motor_current: 1.1[A]");
        println!("{}", s);

        let s = format!("{}", b);
        assert_eq!(s, "motor2 [A] Referrer: Not Connected!");
        println!("{}", s);

        b.connect_to(&a).unwrap();
        let s = format!("{}", b);
        assert_eq!(s, "motor2: 1.1 [A] Referrer: motor_current[A]");
        println!("{}", s);

    }

    #[test]
    fn share_test() {
        let mut a = Signal::new(1.0, "a", "-");
        let mut b = RefSignal::new("b", "-");

        b.connect_to(&a).unwrap();
        assert_eq!(a.val(), b.val());
        println!("a = {}, b = {}", a.val(), b.val());

        a.set_val(2.0);
        assert_eq!(a.val(), b.val());
        println!("a = {}, b = {}", a.val(), b.val());
    }
}