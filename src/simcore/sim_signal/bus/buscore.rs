use std::collections::HashMap;

use core::slice::{Iter, IterMut};
use std::ops::Index;
use std::ops::IndexMut;

use super::super::signal::{SigTrait, SigDef};

use anyhow::{*};



/// Signalを複数まとめたものをBusと定義する。
/// シミュレーションで作るモデル同士のインターフェースとして用いる。
/// BusとRefBus共通メソッドを定義する
#[derive(Debug, Clone)]
pub struct BusCore<T:SigTrait> {
    signals: Vec<T>,
    keytable: HashMap<String, usize>, // 名前からインデックスを検索するためのテーブル
}

impl<T: SigTrait> BusCore<T> {
    pub fn new() -> Self {
        Self {
            signals: Vec::new(),    // 整数型用の信号配列
            keytable: HashMap::new(),
            
        }
    }

    /// BusにSignalを追加する
    pub fn push(&mut self, signal: T) -> anyhow::Result<()> {
        let keyname = signal.name();
        match self.keytable.contains_key(&keyname) {
            true => {
                return Err(anyhow!(format!("信号を追加しようとしましたが信号名が重複しています。: 信号名{}", keyname)));
            },
            false => {
                self.keytable.insert(keyname, self.signals.len());
                self.signals.push(signal);
            }
        } 
        
        Ok(())
    }

    /// 信号名から信号を抽出する
    pub fn get_by_name(&self, signame: impl Into<String>) -> Option<&T> {
        match self.keytable.get(&signame.into()) {
            Some(index) => {
                Some(&self.signals[*index])
            },
            None => None
        }
    }

    /// get_by_nameのmutable版
    pub fn get_by_name_mut(&mut self, signame: impl Into<String>) -> Option<&mut T> {
        match self.keytable.get(&signame.into()) {
            Some(index) => {
                Some(&mut self.signals[*index])
            },
            None => None
        }
    }
    
    /// Signalsのvalue値を集めたVecを返す。データの順番は保持
    pub fn to_vec_f64(&self) -> Vec<f64> {
        self.signals.iter().map(|sig| sig.val()).collect::<Vec<f64>>()
    }

    /// Vec<SigDef>を返す。モデルの作成時に使用する。
    pub fn get_sigdef(&self) -> Vec<SigDef> {
        self.signals.iter().map(|sig| SigDef::new(sig.name().to_string(), sig.unit().to_string()))
                            .collect::<Vec<SigDef>>()
    }

    /// イテレータ
    pub fn iter(&self) -> BusIter<T> {
        self.signals.iter()
    }

    /// イテレータ
    pub fn iter_mut(&mut self) -> BusIterMut<T> {
        self.signals.iter_mut()
    }

    /// 要素数を返す
    pub fn len(&self) -> usize {
        self.signals.len()
    }

}

/// Busのイテレータオブジェクト実装
type BusIter<'a, T> = Iter<'a, T>;
type BusIterMut<'a, T> = IterMut<'a, T>;

/// Indexオペレータのオーバーロード
impl<T: SigTrait> Index<usize> for BusCore<T>
{
    type Output = T;
    fn index(&self, index: usize) -> &Self::Output {
        &self.signals[index]
    }
}
impl<T: SigTrait> IndexMut<usize> for BusCore<T>
{
    fn index_mut(&mut self, index: usize) -> &mut Self::Output {
        &mut self.signals[index]
    }
}

#[cfg(test)]
mod buscore_test {
    use super::*;
    use super::super::super::signal::{*};

    #[test]
    fn buscore_pushtest() {
        let mut a = BusCore::new();

        a.push(Signal::new(1.0, "test1", "A")).unwrap();
        a.push(Signal::new(2.0, "test2", "A")).unwrap();

        assert_eq!(a.get_by_name("test1").unwrap().val(), 1.0);
        assert_eq!(a.get_by_name("test2").unwrap().val(), 2.0);
    }

    #[test]
    #[should_panic]
    fn buscore_pushtest2() {
        let mut  a = BusCore::new();

        a.push(Signal::new(1.0, "test1", "A")).unwrap();
        a.push(Signal::new(2.0, "test1", "A")).unwrap();
    }

    #[test]
    fn buscore_setvalue() {
        let mut  a  = BusCore::new();

        a.push(Signal::new(1.0, "test1", "A")).unwrap();
        a.push(Signal::new(2.0, "test2", "A")).unwrap();

        assert_eq!(a.get_by_name("test1").unwrap().val(), 1.0);

        a.get_by_name_mut("test1").unwrap().set_val(2.0);

        assert_eq!(a.get_by_name("test1").unwrap().val(), 2.0);

    }

    #[test]
    #[should_panic]
    fn buscore_setvalue2() {
        let mut  a  = BusCore::new();

        a.push(Signal::new(1.0, "test1", "A")).unwrap();
        a.push(Signal::new(2.0, "test2", "A")).unwrap();

        a.get_by_name_mut("test3").unwrap().set_val(2.0);

    }

    #[test]
    fn buscore_mk_vec() {
        let mut  a  = BusCore::new();

        a.push(Signal::new(1.0, "test1", "A")).unwrap();
        a.push(Signal::new(2.0, "test2", "A")).unwrap();
        a.push(Signal::new(3.0, "test3", "A")).unwrap();

        assert_eq!(a.to_vec_f64(), vec![1.0, 2.0, 3.0]);

    }

    #[test]
    fn buscore_get_sigdef() {
        let mut  a  = BusCore::new();

        a.push(Signal::new(1.0, "test1", "A")).unwrap();
        a.push(Signal::new(2.0, "test2", "A")).unwrap();

        let sigdef = a.get_sigdef();

        assert_eq!(sigdef, vec![SigDef::new("test1", "A"), SigDef::new("test2", "A")]);

    }

    #[test]
    fn buscore_iter() {
        let mut  a  = BusCore::new();

        a.push(Signal::new(1.0, "test1", "A")).unwrap();
        a.push(Signal::new(2.0, "test2", "A")).unwrap();
        a.push(Signal::new(3.0, "test3", "A")).unwrap();

        let v = a.iter().map(|sig| sig.val() * 2.0).collect::<Vec<f64>>();

        assert_eq!(v, vec![2.0, 4.0, 6.0]);

    }

    #[test]
    fn buscore_iter_mut() {
        let mut  a  = BusCore::new();

        a.push(Signal::new(1.0, "test1", "A")).unwrap();
        a.push(Signal::new(2.0, "test2", "A")).unwrap();
        a.push(Signal::new(3.0, "test3", "A")).unwrap();

        a.iter_mut().for_each(|sig| sig.set_val(10.0));

        assert_eq!(a.to_vec_f64(), vec![10.0, 10.0, 10.0]);

    }

    #[test]
    fn buscore_index() {
        let mut  a  = BusCore::new();

        a.push(Signal::new(1.0, "test1", "A")).unwrap();
        a.push(Signal::new(2.0, "test2", "A")).unwrap();
        a.push(Signal::new(3.0, "test3", "A")).unwrap();

        assert_eq!(a[0].name(), "test1");
        assert_eq!(a[1].name(), "test2");
        assert_eq!(a[2].name(), "test3");
    }

    #[test]
    fn buscore_index_mut() {
        let mut  a  = BusCore::new();

        a.push(Signal::new(1.0, "test1", "A")).unwrap();
        a.push(Signal::new(2.0, "test2", "A")).unwrap();
        a.push(Signal::new(3.0, "test3", "A")).unwrap();

        a[0].set_val(2.0);
        a[1].set_val(3.0);
        a[2].set_val(4.0);

        assert_eq!(a.get_by_name("test1").unwrap().val(), 2.0);
        assert_eq!(a.get_by_name("test2").unwrap().val(), 3.0);
        assert_eq!(a.get_by_name("test3").unwrap().val(), 4.0);

    }
}