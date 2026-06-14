use std::path::Path;

use memmap2::Mmap;
use thiserror::Error;

use crate::mfl_format::{
    decode_signed_sample, FileHeader, FrameHeader, FILE_HEADER_SIZE, FRAME_HEADER_SIZE,
};

#[derive(Error, Debug)]
#[allow(dead_code)]
pub enum ParseError {
    #[error("IO error: {0}")]
    Io(#[from] std::io::Error),
    #[error("invalid magic bytes – not an MFL file")]
    BadMagic,
    #[error("file too short: got {got} bytes, need at least {need}")]
    Truncated { got: usize, need: usize },
    #[error("frame {index} at offset {offset} is malformed: {reason}")]
    BadFrame {
        index: usize,
        offset: usize,
        reason: String,
    },
}

pub struct MflStream {
    mmap: Mmap,
    header: FileHeader,
}

pub struct Frame<'a> {
    pub hdr: FrameHeader,
    pub samples: &'a [u8],
    #[allow(dead_code)]
    pub header: &'a FileHeader,
}

impl MflStream {
    pub fn open(path: &Path) -> Result<Self, ParseError> {
        let file = std::fs::File::open(path)?;
        let metadata = file.metadata()?;
        if metadata.len() < FILE_HEADER_SIZE as u64 {
            return Err(ParseError::Truncated {
                got: metadata.len() as usize,
                need: FILE_HEADER_SIZE,
            });
        }
        let mmap = unsafe { Mmap::map(&file)? };
        let header = FileHeader::parse(&mmap[..FILE_HEADER_SIZE]).ok_or(ParseError::BadMagic)?;
        Ok(Self { mmap, header })
    }

    pub fn header(&self) -> &FileHeader {
        &self.header
    }

    pub fn num_frames(&self) -> usize {
        let data_len = self.mmap.len() - FILE_HEADER_SIZE;
        data_len / self.header.frame_total_bytes()
    }

    pub fn frame(&self, idx: usize) -> Option<Frame<'_>> {
        let offset = FILE_HEADER_SIZE + idx * self.header.frame_total_bytes();
        let end = offset + self.header.frame_total_bytes();
        if end > self.mmap.len() {
            return None;
        }
        let hdr_buf = &self.mmap[offset..offset + FRAME_HEADER_SIZE];
        let hdr = FrameHeader::parse(hdr_buf)?;
        let data_start = offset + FRAME_HEADER_SIZE;
        let data_end = data_start + self.header.frame_data_bytes();
        let samples = &self.mmap[data_start..data_end];
        Some(Frame {
            hdr,
            samples,
            header: &self.header,
        })
    }

    #[allow(dead_code)]
    pub fn iter_frames(&self) -> FrameIter<'_> {
        FrameIter {
            stream: self,
            pos: 0,
        }
    }

    pub fn decode_channel_axes(
        &self,
        frame: &Frame<'_>,
        channel: u16,
    ) -> Vec<i32> {
        let n_axes = self.header.num_axes as usize;
        let sbytes = self.header.sample_bytes();
        let base = channel as usize * n_axes * sbytes;
        let bits = self.header.sample_resolution_bits;
        let mut out = Vec::with_capacity(n_axes);
        for a in 0..n_axes {
            let off = base + a * sbytes;
            if off + sbytes <= frame.samples.len() {
                out.push(decode_signed_sample(
                    &frame.samples[off..off + sbytes],
                    bits,
                ));
            }
        }
        out
    }
}

#[allow(dead_code)]
pub struct FrameIter<'a> {
    stream: &'a MflStream,
    pos: usize,
}

impl<'a> Iterator for FrameIter<'a> {
    type Item = Frame<'a>;

    fn next(&mut self) -> Option<Self::Item> {
        let f = self.stream.frame(self.pos)?;
        self.pos += 1;
        Some(f)
    }

    fn size_hint(&self) -> (usize, Option<usize>) {
        let rem = self.stream.num_frames().saturating_sub(self.pos);
        (rem, Some(rem))
    }
}

impl<'a> ExactSizeIterator for FrameIter<'a> {}
