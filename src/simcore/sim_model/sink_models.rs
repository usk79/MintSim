/// # Sinkモデル
/// Sinkモデルには、下記のモデルを実装する
/// 
/// - Recorderモデル
use super::model_core::{ModelCore};

use super::super::sim_signal;
use sim_signal::signal::{SigTrait};

use sim_signal::bus::{Bus, RefBus};

use super::super::sim_system;
use sim_system::SimTime;

use plotters::prelude::*;
use plotters::coord::Shift;

use std::fs::File;
use std::io::{Write, BufWriter};

use anyhow::{anyhow};

#[derive(Debug)]
pub struct SimRecorder {
    timedata: Vec<f64>,     // 時刻情報保管用
    storage: Vec<Vec<f64>>, // データストレージ
    signum: usize,
    input_bus: RefBus, 
}

impl SimRecorder {
    pub fn new(inbus: RefBus) -> Self {
        Self {
            timedata: Vec::new(),
            storage: Vec::new(),
            signum: inbus.len(),
            input_bus: inbus,
        }
    }

    pub fn export(&self, filepath: &str) -> anyhow::Result<()> {
        let mut file = BufWriter::new(File::create(filepath).unwrap());
        
        // 一行目の信号名の部分を作成
        let mut seriesname = vec!["time[s]".to_string()];
        self.input_bus.get_sigdef().iter().for_each(|sig| seriesname.push( sig.to_string() ) );
        
        writeln!(file, "{}", seriesname.join(","))?;

        // データを1行ずつ出力
        let siglen = self.timedata.len();
        
        for idx in 0..siglen {
            let mut line = vec![self.timedata[idx].to_string()];

            for sigidx in 0..self.signum {
                line.push(self.storage[sigidx][idx].to_string());
            }

            writeln!(file, "{}", line.join(","))?;
        }

        Ok(())
    }

    pub fn timeplot_all(&self, filename: &str, pltsize: (u32, u32), pltdivide: (usize, usize)) -> anyhow::Result<()>{
        let root_area = BitMapBackend::new(filename, pltsize).into_drawing_area();
        let child_areas = root_area.split_evenly(pltdivide);

        root_area.fill(&WHITE).unwrap();

        if self.signum > pltdivide.0 * pltdivide.1 {
            return Err(anyhow!("プロットの分割数が不足しています。"));
        }

        self.storage.iter().enumerate().for_each( |(idx, data)| {
            let sig = &self.input_bus.get_sigdef()[idx];
            self.timeplot_subfn(&child_areas[idx], &sig.to_string(), &data);
        });

        Ok(())
    }

    fn timeplot_subfn(&self, plt: &DrawingArea<BitMapBackend, Shift>, caption: &str, data: &Vec<f64>) {
        
        plt.fill(&WHITE).unwrap();
    
        let font = ("sans-serif", 20);

        let (y_min, y_max) = data.iter()
                         .fold(
                           (0.0/0.0, 0.0/0.0),
                           |(m,n), v| (v.min(m), v.max(n))
                          ); // f64はNaNがあるためordが実装されていない。min, maxを使うための工夫が必要⇒https://qiita.com/lo48576/items/343ca40a03c3b86b67cb
        let datalen = self.timedata.len();
        let xrange = 0.0..self.timedata[datalen - 1]; 
        let yrange = y_min..y_max;
      
        let mut chart = ChartBuilder::on(&plt)
          .caption(caption, font.into_font()) // キャプションのフォントやサイズ
          .margin(10)                         // 上下左右全ての余白
          .x_label_area_size(16)              // x軸ラベル部分の余白
          .y_label_area_size(42)              // y軸ラベル部分の余白
          .build_cartesian_2d(                // x軸とy軸の数値の範囲を指定する
            xrange,                           // x軸の範囲
            yrange)                           // y軸の範囲
          .unwrap();
    
        // x軸y軸、グリッド線などを描画
        chart.configure_mesh().draw().unwrap();

        let line_series = LineSeries::new(
            self.timedata.iter()
                    .zip(data.iter())
                    .map(|(x, y)| (*x, *y)),
                &RED);

        chart.draw_series(line_series).unwrap();

    }
}

impl ModelCore for SimRecorder {
    fn initialize(&mut self, sim_time: &SimTime) {
        let stepnum = sim_time.step_num();

        self.timedata = Vec::with_capacity(stepnum);
        self.storage = (0..self.signum).map(|_| Vec::with_capacity(stepnum) ).collect::<Vec<Vec<f64>>>();
        self.storage.iter_mut().enumerate().for_each(|(idx, sig)| sig.push(self.input_bus[idx].val())); // 0秒のデータはbusの初期値を入れる（たいていの場合は0)
        
        self.timedata.push(sim_time.start_time()); // 初期時間を設定する
        
    }

    fn finalize(&mut self) {
        // 処理なし
    }

    fn nextstate(&mut self, sim_time: &SimTime) {
        // 処理なし
        self.timedata.push(sim_time.time());
        
        self.input_bus.iter().enumerate().for_each(|(idx, sig)| {
            self.storage[idx].push(sig.val());
        })

    }

    fn interface_in(&mut self) -> Option<&mut RefBus> {
        Some(&mut self.input_bus)
    }

    fn interface_out(&self) -> Option<&Bus> {
        None
    }
}



#[cfg(test)]
mod scope_test {

    use super::*;
    use sim_signal::signal::{SigDef};
    #[test]
    fn scope_pushtest() {
        let mut bus = Bus::try_from(vec![
            SigDef::new("motor_trq", "Nm"),
            SigDef::new("motor_volt", "V"),
            SigDef::new("motor_current", "A"),
        ]).unwrap();

        let mut inbus = RefBus::try_from(vec![
            SigDef::new("motor_trq", "Nm"),
            SigDef::new("motor_volt", "V"),
            SigDef::new("motor_current", "A"),
        ]).unwrap();

        inbus.connect_to(&bus, &["motor_trq", "motor_volt", "motor_current"], &["motor_trq", "motor_volt", "motor_current"]).unwrap();
        

        let mut scope = SimRecorder::new(inbus);
        let mut sim_time = SimTime::new(0.0, 1.0, 0.001);
        
        bus[0].set_val(1.0);
        bus[1].set_val(2.0);
        
        scope.initialize(&sim_time);

        assert_eq!(scope.storage[0][0], 1.0);
        assert_eq!(scope.storage[1][0], 2.0);
        assert_eq!(scope.storage[2][0], 0.0);

        while let Some((i, _t)) = sim_time.next() { 

            bus[0].set_val(i as f64);
            bus[1].set_val((i * 2) as f64);
            bus[2].set_val((i * 3) as f64);

            scope.nextstate(&sim_time);
        }

        scope.export("test_output\\scope_pushtest.csv").unwrap();
        scope.timeplot_all("test_output\\scope_pushtest.png", (500, 500), (4, 1)).unwrap();

        assert_eq!(scope.storage[0][10], 10.0);
        assert_eq!(scope.storage[1][10], 20.0);
        assert_eq!(scope.storage[2][10], 30.0);

        
    }
}
