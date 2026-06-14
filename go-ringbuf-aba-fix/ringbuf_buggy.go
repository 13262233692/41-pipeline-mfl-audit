package ringbuf

import (
	"runtime"
	"sync/atomic"
)

type buggySlot struct {
	val ContactEvent
}

type RingBufferBuggy struct {
	buf   []buggySlot
	cap   uint64
	mask  uint64
	head  uint64
	_     [7]uint64
	tail  uint64
	_     [7]uint64
}

func NewRingBufferBuggy(capacity int) *RingBufferBuggy {
	if capacity <= 0 {
		capacity = 1024
	}
	cap := nextPowerOfTwo(uint64(capacity))
	rb := &RingBufferBuggy{
		cap:  cap,
		mask: cap - 1,
		buf:  make([]buggySlot, cap),
	}
	return rb
}

func (rb *RingBufferBuggy) Enqueue(val *ContactEvent) bool {
	for {
		head := atomic.LoadUint64(&rb.head)
		tail := atomic.LoadUint64(&rb.tail)

		if tail-head >= rb.cap {
			return false
		}

		idx := tail & rb.mask
		if atomic.CompareAndSwapUint64(&rb.tail, tail, tail+1) {
			rb.buf[idx].val = *val
			return true
		}
		runtime.Gosched()
	}
}

func (rb *RingBufferBuggy) Dequeue(val *ContactEvent) bool {
	for {
		head := atomic.LoadUint64(&rb.head)
		tail := atomic.LoadUint64(&rb.tail)

		if head == tail {
			return false
		}

		idx := head & rb.mask
		*val = rb.buf[idx].val

		if atomic.CompareAndSwapUint64(&rb.head, head, head+1) {
			return true
		}
		runtime.Gosched()
	}
}

func (rb *RingBufferBuggy) Size() int {
	head := atomic.LoadUint64(&rb.head)
	tail := atomic.LoadUint64(&rb.tail)
	return int(tail - head)
}

func (rb *RingBufferBuggy) Capacity() int {
	return int(rb.cap)
}

func (rb *RingBufferBuggy) IsEmpty() bool {
	return atomic.LoadUint64(&rb.head) == atomic.LoadUint64(&rb.tail)
}
