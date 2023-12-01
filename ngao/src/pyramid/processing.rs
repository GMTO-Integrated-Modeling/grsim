use std::ops::{Add, DivAssign, Mul, Sub};

use crate::Processing;

use super::{PyramidData, PyramidProcessor};

impl<T> Processing for PyramidProcessor<T>
where
    T: std::fmt::Debug
        + TryFrom<f32>
        + Copy
        + Add<Output = T>
        + Sub<Output = T>
        + DivAssign
        + Mul<Output = T>
        + PartialOrd
        + 'static,
    for<'a> &'a T: Sub<Output = T> + Add<Output = T>,
    <T as TryFrom<f32>>::Error: std::fmt::Debug,
{
    type ProcessorData = PyramidData<T>;

    fn processing(&self) -> Self::ProcessorData {
        let (n, m) = self.frame.resolution;
        assert!(n > 0 && m > 0, "the detector frame resolution is null");

        let gmt_dos_clients_crseo::crseo::wavefrontsensor::LensletArray { n_side_lenslet, .. } =
            self.lenslet_array;
        let n0 = n_side_lenslet / 2;
        let n1 = n0 + n / 2;

        /*
        The detector frame of the pyramid is divided in 4 quadrants:
        | I1 I2 |
        | I3 I4 |
         */

        /* left =
        | I1 |
        | I3 |
         */
        let left = self.frame.value.chunks(n).skip(n0).take(n_side_lenslet);
        /* right =
        | I2 |
        | I4 |
         */
        let right = self.frame.value.chunks(n).skip(n1).take(n_side_lenslet);
        // (left - right, left + right)
        let row_diff_sum: Vec<_> = left
            .zip(right)
            .flat_map(|(left, right)| {
                left.iter()
                    .zip(right)
                    .map(|(left, right)| (left - right, left + right))
                    .collect::<Vec<_>>()
            })
            .collect();
        // top = ( I1 - I2 , I1 + I2 )
        let top = (0..n)
            .map(|i| row_diff_sum.iter().skip(i).step_by(m).collect::<Vec<_>>())
            .skip(n0)
            .take(n_side_lenslet);
        // bottom = ( I3 -I4 , I3 + I4)
        let bottom = (0..n)
            .map(|i| row_diff_sum.iter().skip(i).step_by(m).collect::<Vec<_>>())
            .skip(n1)
            .take(n_side_lenslet);
        /*
        top - bottom = I1 - I2 + I3 - I4
        top + bottom = I1 + I2 + I3 + I4
         */
        let (row_col_data, mut flux): (Vec<_>, Vec<_>) = top
            .zip(bottom)
            .flat_map(|(top, bottom)| {
                top.iter()
                    .zip(bottom)
                    .map(|((top_diff, top_sum), (bottom_diff, bottom_sum))| {
                        (*top_diff + *bottom_diff, *top_sum + *bottom_sum)
                    })
                    .collect::<Vec<_>>()
            })
            .unzip();

        // top = | I1 I2 |
        let top = (0..n)
            .map(|i| {
                self.frame
                    .value
                    .iter()
                    .skip(i)
                    .step_by(m)
                    .collect::<Vec<_>>()
            })
            .skip(n0)
            .take(n_side_lenslet);
        // bottom = | I3 I4 |
        let bottom = (0..n)
            .map(|i| {
                self.frame
                    .value
                    .iter()
                    .skip(i)
                    .step_by(m)
                    .collect::<Vec<_>>()
            })
            .skip(n1)
            .take(n_side_lenslet);
        // top - bottom
        let col_diff: Vec<_> = top
            .zip(bottom)
            .flat_map(|(top, bottom)| {
                top.iter()
                    .zip(bottom)
                    .map(|(&&top, &bottom)| top - bottom)
                    .collect::<Vec<_>>()
            })
            .collect();
        // I1 - I3 + I2 - I4
        let left = col_diff.chunks(n);
        let right = col_diff.chunks(n);
        let col_row_data: Vec<_> = left
            .zip(right)
            .flat_map(|(left, right)| {
                left.iter()
                    .skip(n0)
                    .take(n_side_lenslet)
                    .zip(right.iter().skip(n1).take(n_side_lenslet))
                    .map(|(&left, &right)| left + right)
                    .collect::<Vec<_>>()
            })
            .collect();

        flux.sort_by(|a, b| a.partial_cmp(b).unwrap());
        let med_flux = match flux.len() {
            n if n % 2 == 0 => T::try_from(0.5).unwrap() * (flux[n / 2] + flux[1 + n / 2]),
            n => flux[(n + 1) / 2],
        };

        let (sx, sy): (Vec<_>, Vec<_>) = row_col_data
            .into_iter()
            .zip(col_row_data.into_iter())
            .map(|(mut sx, mut sy)| {
                sx /= med_flux;
                sy /= med_flux;
                (sx, sy)
            })
            .unzip();

        /*         let (sx, sy): (Vec<_>, Vec<_>) = row_col_data
        .into_iter()
        .zip(col_row_data.into_iter())
        .zip(flux.iter())
        .map(|((mut sx, mut sy), flux)| {
            sx /= *flux;
            sy /= *flux;
            (sx, sy)
        })
        .unzip(); */

        PyramidData { sx, sy, flux }
    }
}
