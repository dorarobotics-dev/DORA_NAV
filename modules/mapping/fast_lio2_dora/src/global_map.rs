use anyhow::{Context, Result, ensure};
use fast_lio::types::{PointCloud, PointType};
use std::{
    collections::HashMap,
    fs::File,
    io::{BufWriter, Write},
    path::Path,
};

/// Accumulates registered scans independently of the core's sliding local map.
pub struct GlobalMap {
    cells: HashMap<(i64, i64, i64), PointType>,
    voxel: f64,
    limit: usize,
}
impl GlobalMap {
    pub fn new(voxel: f32, limit: usize) -> Self {
        Self {
            cells: HashMap::new(),
            voxel: voxel as f64,
            limit,
        }
    }
    pub fn insert(&mut self, points: &[PointType]) -> Result<()> {
        for p in points {
            ensure!(
                [p.x, p.y, p.z, p.intensity].iter().all(|v| v.is_finite()),
                "nonfinite registered scan"
            );
            let cell = |v: f32| -> Result<i64> {
                let c = (v as f64 / self.voxel).floor();
                ensure!(
                    c.is_finite() && c.abs() < i64::MAX as f64,
                    "map voxel index overflow"
                );
                Ok(c as i64)
            };
            let key = (cell(p.x)?, cell(p.y)?, cell(p.z)?);
            if !self.cells.contains_key(&key) {
                ensure!(
                    self.cells.len() < self.limit,
                    "global map reached output.max_points; increase limit or stop mapping"
                );
                self.cells.insert(key, *p);
            }
        }
        Ok(())
    }
    pub fn len(&self) -> usize {
        self.cells.len()
    }
    pub fn points(&self) -> PointCloud {
        self.cells.values().copied().collect()
    }

    pub fn save(&self, path: &Path) -> Result<()> {
        ensure!(!self.cells.is_empty(), "no initialized map to save");
        let parent = path.parent().unwrap_or(Path::new("."));
        std::fs::create_dir_all(parent)
            .with_context(|| format!("create map directory {}", parent.display()))?;
        let name = path
            .file_name()
            .context("map path needs a filename")?
            .to_string_lossy();
        let temp = parent.join(format!(".{name}.tmp-{}", std::process::id()));
        let result = (|| -> Result<()> {
            let mut out = BufWriter::new(File::create(&temp)?);
            writeln!(
                out,
                "# .PCD v0.7 - FAST-LIO2 accumulated world voxel map\nVERSION 0.7\nFIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\nWIDTH {}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {}\nDATA binary",
                self.len(),
                self.len()
            )?;
            for p in self.cells.values() {
                for v in [p.x, p.y, p.z, p.intensity] {
                    out.write_all(&v.to_le_bytes())?;
                }
            }
            out.flush()?;
            out.get_ref().sync_all()?;
            drop(out);
            std::fs::rename(&temp, path)?;
            Ok(())
        })();
        if result.is_err() {
            let _ = std::fs::remove_file(&temp);
        }
        result.with_context(|| format!("save map {}", path.display()))
    }
}
