use std::{error::Error, path::Path, sync::Arc};

use colorous::CIVIDIS;
use image::RgbImage;

#[derive(Default, Debug, Clone)]
pub struct Heatmap {
    size: usize,
    map: Arc<Vec<f64>>,
    buffer: RgbImage,
}

impl Heatmap {
    pub fn new(size: usize) -> Self {
        Self {
            size,
            ..Default::default()
        }
    }
    pub fn map(mut self, map: &Arc<Vec<f64>>) -> Self {
        self.map = map.clone();
        self
    }
    pub fn save<P: AsRef<Path>>(self, path: P) -> Result<(), Box<dyn Error>> {
        let max_px = self
            .map
            .iter()
            .max_by(|&a, &b| a.partial_cmp(b).unwrap())
            .unwrap();
        let min_px = self
            .map
            .iter()
            .min_by(|&a, &b| a.partial_cmp(b).unwrap())
            .unwrap();
        let colormap = CIVIDIS;
        let pixels: Vec<_> = self
            .map
            .iter()
            .map(|x| (*x - min_px) / (max_px - min_px))
            .map(|t| colormap.eval_continuous(t.into()))
            .flat_map(|c| c.into_array().to_vec())
            .collect();
        let height = self.size;
        let width = self.map.len() / height;

        /* let pixels: Vec<_> = (0..self.size)
        .flat_map(|i| {
            pixels
                .chunks(self.size)
                .step_by(self.size + i)
                .flat_map(|c| c.to_vec())
                .collect::<Vec<_>>()
        })
        .collect(); */

        RgbImage::from_vec(width as u32, height as u32, pixels)
            .as_ref()
            .map(|buffer| buffer.save(path))
            .transpose()?
            .unwrap();
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn heatmap() {
        let n = 32;
        let mut map = vec![0f64; n * n];
        map[0] = 1.;
        Heatmap::new(n)
            .map(&Arc::new(map))
            .save("heatmap.png")
            .unwrap();
    }
}
