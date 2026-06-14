use std::cell::UnsafeCell;
use std::marker::PhantomData;
use std::sync::atomic::{AtomicU64, Ordering};

const CACHE_LINE_BYTES: usize = 64;

fn next_power_of_two(n: u64) -> u64 {
    if n <= 1 {
        return 2;
    }
    let mut n = n - 1;
    n |= n >> 1;
    n |= n >> 2;
    n |= n >> 4;
    n |= n >> 8;
    n |= n >> 16;
    n |= n >> 32;
    n + 1
}

#[repr(C, align(64))]
struct CachePaddedAtomicU64 {
    val: AtomicU64,
    _pad: [u8; CACHE_LINE_BYTES - std::mem::size_of::<AtomicU64>()],
}

impl CachePaddedAtomicU64 {
    fn new(v: u64) -> Self {
        Self {
            val: AtomicU64::new(v),
            _pad: [0u8; CACHE_LINE_BYTES - std::mem::size_of::<AtomicU64>()],
        }
    }
}

struct Slot<T> {
    seq: AtomicU64,
    val: UnsafeCell<T>,
}

unsafe impl<T: Send> Send for Slot<T> {}
unsafe impl<T: Send> Sync for Slot<T> {}

pub struct RingBuffer<T: Copy + Send> {
    buf_ptr: *mut Slot<T>,
    cap: u64,
    mask: u64,
    head: CachePaddedAtomicU64,
    tail: CachePaddedAtomicU64,
    _marker: PhantomData<T>,
}

unsafe impl<T: Copy + Send> Send for RingBuffer<T> {}
unsafe impl<T: Copy + Send> Sync for RingBuffer<T> {}

impl<T: Copy + Send> RingBuffer<T> {
    pub fn new(capacity: usize) -> Self {
        let cap = next_power_of_two(capacity.max(2) as u64);
        let mut slots: Vec<Slot<T>> = (0..cap)
            .map(|i| Slot {
                seq: AtomicU64::new(i),
                val: UnsafeCell::new(unsafe { std::mem::zeroed() }),
            })
            .collect();
        let buf_ptr = slots.as_mut_ptr();
        std::mem::forget(slots);

        Self {
            buf_ptr,
            cap,
            mask: cap - 1,
            head: CachePaddedAtomicU64::new(0),
            tail: CachePaddedAtomicU64::new(0),
            _marker: PhantomData,
        }
    }

    #[inline(always)]
    unsafe fn slot_at(&self, idx: u64) -> *mut Slot<T> {
        self.buf_ptr.add(idx as usize)
    }

    pub fn enqueue(&self, val: T) -> bool {
        loop {
            let tail = self.tail.val.load(Ordering::Acquire);
            let idx = tail & self.mask;
            let slot_ptr = unsafe { self.slot_at(idx) };
            let seq = unsafe { (*slot_ptr).seq.load(Ordering::Acquire) };

            if seq == tail {
                if self
                    .tail
                    .val
                    .compare_exchange_weak(tail, tail + 1, Ordering::AcqRel, Ordering::Acquire)
                    .is_ok()
                {
                    unsafe {
                        std::ptr::write((*slot_ptr).val.get(), val);
                    }
                    unsafe {
                        (*slot_ptr).seq.store(tail + 1, Ordering::Release);
                    }
                    return true;
                }
            } else if seq < tail {
                return false;
            }
            std::hint::spin_loop();
        }
    }

    pub fn dequeue(&self) -> Option<T> {
        loop {
            let head = self.head.val.load(Ordering::Acquire);
            let idx = head & self.mask;
            let slot_ptr = unsafe { self.slot_at(idx) };
            let seq = unsafe { (*slot_ptr).seq.load(Ordering::Acquire) };

            let expected = head + 1;
            if seq == expected {
                if self
                    .head
                    .val
                    .compare_exchange_weak(head, head + 1, Ordering::AcqRel, Ordering::Acquire)
                    .is_ok()
                {
                    let val = unsafe { std::ptr::read((*slot_ptr).val.get()) };
                    unsafe {
                        (*slot_ptr).seq.store(head + self.cap, Ordering::Release);
                    }
                    return Some(val);
                }
            } else if seq < expected {
                return None;
            }
            std::hint::spin_loop();
        }
    }

    pub fn capacity(&self) -> usize {
        self.cap as usize
    }

    pub fn len(&self) -> usize {
        let head = self.head.val.load(Ordering::Acquire);
        let tail = self.tail.val.load(Ordering::Acquire);
        tail.saturating_sub(head) as usize
    }

    pub fn is_empty(&self) -> bool {
        self.head.val.load(Ordering::Acquire)
            == self.tail.val.load(Ordering::Acquire)
    }
}

impl<T: Copy + Send> Drop for RingBuffer<T> {
    fn drop(&mut self) {
        let cap = self.cap as usize;
        let head = self.head.val.load(Ordering::Acquire);
        let tail = self.tail.val.load(Ordering::Acquire);
        for pos in head..tail {
            let idx = (pos & self.mask) as usize;
            unsafe {
                let slot_ptr = self.buf_ptr.add(idx);
                std::ptr::drop_in_place((*slot_ptr).val.get());
            }
        }
        unsafe {
            Vec::from_raw_parts(self.buf_ptr, cap, cap);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::Arc;
    use std::thread;

    #[derive(Copy, Clone, Debug, PartialEq, Eq)]
    struct Payload {
        id: u32,
        val: u64,
    }

    #[test]
    fn spsc_basic() {
        let rb = RingBuffer::<u32>::new(8);
        assert!(rb.is_empty());
        assert!(rb.dequeue().is_none());
        assert!(rb.enqueue(42));
        assert_eq!(rb.len(), 1);
        assert_eq!(rb.dequeue(), Some(42));
        assert!(rb.is_empty());
    }

    #[test]
    fn spsc_sequencing() {
        let rb = Arc::new(RingBuffer::<Payload>::new(16));
        const N: u32 = 100_000;

        let rb_prod = rb.clone();
        let prod = thread::spawn(move || {
            for i in 0..N {
                let p = Payload { id: i, val: i as u64 * 2 };
                while !rb_prod.enqueue(p) {
                    std::hint::spin_loop();
                }
            }
        });

        let rb_cons = rb.clone();
        let cons = thread::spawn(move || {
            let mut seen = Vec::with_capacity(N as usize);
            while seen.len() < N as usize {
                if let Some(p) = rb_cons.dequeue() {
                    seen.push(p);
                } else {
                    std::hint::spin_loop();
                }
            }
            seen
        });

        prod.join().unwrap();
        let seen = cons.join().unwrap();
        assert_eq!(seen.len() as u32, N);
        for (i, p) in seen.iter().enumerate() {
            assert_eq!(p.id, i as u32);
            assert_eq!(p.val, (i as u64) * 2);
        }
    }

    #[test]
    fn mpmc_no_duplicates() {
        let rb = Arc::new(RingBuffer::<Payload>::new(256));
        const NUM_PROD: u32 = 8;
        const NUM_CONS: u32 = 4;
        const OPS: u32 = 10_000;
        let total = NUM_PROD * OPS;

        let produced = Arc::new(AtomicU64::new(0));
        let consumed = Arc::new(AtomicU64::new(0));
        let stop = Arc::new(AtomicU64::new(0));

        let mut handles = Vec::new();

        for p in 0..NUM_PROD {
            let rb = rb.clone();
            let prod = produced.clone();
            handles.push(thread::spawn(move || {
                for i in 0..OPS {
                    let id = p * OPS + i;
                    let pl = Payload { id, val: p as u64 };
                    while !rb.enqueue(pl) {
                        std::hint::spin_loop();
                    }
                    prod.fetch_add(1, Ordering::Relaxed);
                }
            }));
        }

        let results: Arc<std::sync::Mutex<Vec<Payload>>> =
            Arc::new(std::sync::Mutex::new(Vec::new()));

        for _ in 0..NUM_CONS {
            let rb = rb.clone();
            let cons = consumed.clone();
            let results = results.clone();
            let stop = stop.clone();
            handles.push(thread::spawn(move || loop {
                if let Some(pl) = rb.dequeue() {
                    results.lock().unwrap().push(pl);
                    let c = cons.fetch_add(1, Ordering::Relaxed) + 1;
                    if c >= total as u64 {
                        stop.store(1, Ordering::Release);
                    }
                } else if stop.load(Ordering::Acquire) == 1 {
                    break;
                } else {
                    std::hint::spin_loop();
                }
            }));
        }

        for h in handles {
            h.join().unwrap();
        }

        let r = results.lock().unwrap();
        assert_eq!(r.len() as u64, total as u64);
        let mut seen = std::collections::HashSet::with_capacity(total as usize);
        for p in r.iter() {
            assert!(seen.insert(p.id));
        }
    }
}
