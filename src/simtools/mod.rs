
/// 物理演算でよく使う定数の定義
pub mod simconsts;
/// mintsim共通で使う信号とそれをまとめるsignalbusの定義
pub mod signal;
/// modelトレイトの宣言といくつかのモデルを提供
pub mod simmodel;
/// シミュレーションを実行するためのシミュレータ管理用トレイト
pub mod simrunner;

/// simtoolsには関係ない関数（デバッグ用）
fn type_of<T>(_: T) -> String{
    let a = std::any::type_name::<T>();
    return a.to_string();
  }
