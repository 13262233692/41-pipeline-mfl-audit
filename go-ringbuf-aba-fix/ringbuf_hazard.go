package ringbuf

import (
	"runtime"
	"sync"
	"sync/atomic"
	"unsafe"
)

type hpNode struct {
	next *hpNode
	val  ContactEvent
}

type HazardPointer struct {
	hp []*hpNode
}

var (
	hpMu      sync.Mutex
	hpList    []*HazardPointer
	retiredMu sync.Mutex
	retired   []*hpNode
)

const maxHazardPointers = 256

func newHazardPointer() *HazardPointer {
	hp := &HazardPointer{
		hp: make([]*hpNode, maxHazardPointers),
	}
	hpMu.Lock()
	hpList = append(hpList, hp)
	hpMu.Unlock()
	return hp
}

func (hp *HazardPointer) store(idx int, node *hpNode) {
	atomic.StorePointer((*unsafe.Pointer)(unsafe.Pointer(&hp.hp[idx])), unsafe.Pointer(node))
}

func (hp *HazardPointer) clear(idx int) {
	atomic.StorePointer((*unsafe.Pointer)(unsafe.Pointer(&hp.hp[idx])), nil)
}

func (hp *HazardPointer) load(idx int) *hpNode {
	return (*hpNode)(atomic.LoadPointer((*unsafe.Pointer)(unsafe.Pointer(&hp.hp[idx]))))
}

func isHazardous(node *hpNode) bool {
	hpMu.Lock()
	defer hpMu.Unlock()
	for _, hp := range hpList {
		for i := 0; i < maxHazardPointers; i++ {
			if hp.load(i) == node {
				return true
			}
		}
	}
	return false
}

func retireNode(node *hpNode) {
	retiredMu.Lock()
	retired = append(retired, node)
	retiredMu.Unlock()
	tryReclaim()
}

func tryReclaim() {
	retiredMu.Lock()
	defer retiredMu.Unlock()

	if len(retired) < 1000 {
		return
	}

	var remaining []*hpNode
	for _, node := range retired {
		if isHazardous(node) {
			remaining = append(remaining, node)
		}
	}
	retired = remaining
}

type RingBufferHazard struct {
	head *hpNode
	tail *hpNode
	_    [64 - 16]byte
}

func NewRingBufferHazard() *RingBufferHazard {
	dummy := &hpNode{}
	rb := &RingBufferHazard{
		head: dummy,
		tail: dummy,
	}
	runtime.SetFinalizer(rb, func(*RingBufferHazard) {
		hpMu.Lock()
		for i, hp := range hpList {
			if hp == nil {
				continue
			}
			hpList[i] = nil
		}
		hpMu.Unlock()
	})
	return rb
}

func (rb *RingBufferHazard) Enqueue(val *ContactEvent) bool {
	newNode := &hpNode{val: *val}
	hp := newHazardPointer()

	for {
		t := (*hpNode)(atomic.LoadPointer((*unsafe.Pointer)(unsafe.Pointer(&rb.tail))))
		hp.store(0, t)
		if t != (*hpNode)(atomic.LoadPointer((*unsafe.Pointer)(unsafe.Pointer(&rb.tail)))) {
			continue
		}

		next := (*hpNode)(atomic.LoadPointer((*unsafe.Pointer)(unsafe.Pointer(&t.next))))
		if t != (*hpNode)(atomic.LoadPointer((*unsafe.Pointer)(unsafe.Pointer(&rb.tail)))) {
			continue
		}

		if next == nil {
			if atomic.CompareAndSwapPointer(
				(*unsafe.Pointer)(unsafe.Pointer(&t.next)),
				unsafe.Pointer(next),
				unsafe.Pointer(newNode),
			) {
				atomic.CompareAndSwapPointer(
					(*unsafe.Pointer)(unsafe.Pointer(&rb.tail)),
					unsafe.Pointer(t),
					unsafe.Pointer(newNode),
				)
				hp.clear(0)
				return true
			}
		} else {
			atomic.CompareAndSwapPointer(
				(*unsafe.Pointer)(unsafe.Pointer(&rb.tail)),
				unsafe.Pointer(t),
				unsafe.Pointer(next),
			)
		}
		runtime.Gosched()
	}
}

func (rb *RingBufferHazard) Dequeue(val *ContactEvent) bool {
	hp := newHazardPointer()

	for {
		h := (*hpNode)(atomic.LoadPointer((*unsafe.Pointer)(unsafe.Pointer(&rb.head))))
		hp.store(0, h)
		if h != (*hpNode)(atomic.LoadPointer((*unsafe.Pointer)(unsafe.Pointer(&rb.head)))) {
			continue
		}

		t := (*hpNode)(atomic.LoadPointer((*unsafe.Pointer)(unsafe.Pointer(&rb.tail))))
		next := (*hpNode)(atomic.LoadPointer((*unsafe.Pointer)(unsafe.Pointer(&h.next))))

		if h != (*hpNode)(atomic.LoadPointer((*unsafe.Pointer)(unsafe.Pointer(&rb.head)))) {
			continue
		}

		if next == nil {
			hp.clear(0)
			return false
		}

		if h == t {
			atomic.CompareAndSwapPointer(
				(*unsafe.Pointer)(unsafe.Pointer(&rb.tail)),
				unsafe.Pointer(t),
				unsafe.Pointer(next),
			)
		}

		hp.store(1, next)
		if h != (*hpNode)(atomic.LoadPointer((*unsafe.Pointer)(unsafe.Pointer(&rb.head)))) {
			continue
		}

		if atomic.CompareAndSwapPointer(
			(*unsafe.Pointer)(unsafe.Pointer(&rb.head)),
			unsafe.Pointer(h),
			unsafe.Pointer(next),
		) {
			*val = next.val
			retireNode(h)
			hp.clear(0)
			hp.clear(1)
			return true
		}
		runtime.Gosched()
	}
}

func (rb *RingBufferHazard) IsEmpty() bool {
	h := (*hpNode)(atomic.LoadPointer((*unsafe.Pointer)(unsafe.Pointer(&rb.head))))
	next := (*hpNode)(atomic.LoadPointer((*unsafe.Pointer)(unsafe.Pointer(&h.next))))
	return next == nil
}

func (rb *RingBufferHazard) Size() int {
	count := 0
	h := (*hpNode)(atomic.LoadPointer((*unsafe.Pointer)(unsafe.Pointer(&rb.head))))
	for h != nil {
		next := (*hpNode)(atomic.LoadPointer((*unsafe.Pointer)(unsafe.Pointer(&h.next))))
		if next != nil {
			count++
		}
		h = next
	}
	return count
}
