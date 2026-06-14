package ringbuf

import (
	"runtime"
	"sync/atomic"
)

type ContactEvent struct {
	MachineID uint16
	BondID    uint32
	Timestamp uint64
	Force_mN  float32
	Status    uint8
}

const slotSize = 64

type slot struct {
	seq uint64
	val ContactEvent
}

type RingBufferVersioned struct {
	_     [0]uint64
	buf   []slot
	_     [0]uint64
	cap   uint64
	mask  uint64
	_     [0]uint64
	head  uint64
	_     [0]uint64
	tail  uint64
	_     [0]uint64
}

func NewRingBufferVersioned(capacity int) *RingBufferVersioned {
	if capacity <= 0 {
		capacity = 1024
	}
	cap := nextPowerOfTwo(uint64(capacity))
	rb := &RingBufferVersioned{
		cap:  cap,
		mask: cap - 1,
		buf:  make([]slot, cap),
	}
	for i := uint64(0); i < cap; i++ {
		rb.buf[i].seq = i
	}
	return rb
}

func (rb *RingBufferVersioned) Enqueue(val *ContactEvent) bool {
	for {
		tail := atomic.LoadUint64(&rb.tail)
		idx := tail & rb.mask
		slot := &rb.buf[idx]
		seq := atomic.LoadUint64(&slot.seq)

		if seq == tail {
			if atomic.CompareAndSwapUint64(&rb.tail, tail, tail+1) {
				slot.val = *val
				atomic.StoreUint64(&slot.seq, tail+1)
				return true
			}
		} else if seq < tail {
			return false
		} else {
			runtime.Gosched()
		}
	}
}

func (rb *RingBufferVersioned) Dequeue(val *ContactEvent) bool {
	for {
		head := atomic.LoadUint64(&rb.head)
		idx := head & rb.mask
		slot := &rb.buf[idx]
		seq := atomic.LoadUint64(&slot.seq)

		expected := head + 1
		if seq == expected {
			if atomic.CompareAndSwapUint64(&rb.head, head, head+1) {
				*val = slot.val
				atomic.StoreUint64(&slot.seq, head+rb.cap)
				return true
			}
		} else if seq < expected {
			return false
		} else {
			runtime.Gosched()
		}
	}
}

func (rb *RingBufferVersioned) Size() int {
	head := atomic.LoadUint64(&rb.head)
	tail := atomic.LoadUint64(&rb.tail)
	return int(tail - head)
}

func (rb *RingBufferVersioned) Capacity() int {
	return int(rb.cap)
}

func (rb *RingBufferVersioned) IsEmpty() bool {
	return atomic.LoadUint64(&rb.head) == atomic.LoadUint64(&rb.tail)
}

func nextPowerOfTwo(n uint64) uint64 {
	if n <= 1 {
		return 2
	}
	n--
	n |= n >> 1
	n |= n >> 2
	n |= n >> 4
	n |= n >> 8
	n |= n >> 16
	n |= n >> 32
	n++
	return n
}
