package ringbuf

import (
	"runtime"
	"sync"
	"sync/atomic"
	"testing"
	"time"
)

func TestBuggyABA_DataCorruption(t *testing.T) {
	const numProducers = 8
	const numConsumers = 4
	const opsPerProducer = 50000
	const smallCapacity = 8

	rb := NewRingBufferBuggy(smallCapacity)

	var produced uint64
	var consumed uint64
	var corrupted uint64
	var wg sync.WaitGroup
	var stop int32

	consumer := func(id int) {
		defer wg.Done()
		var val ContactEvent
		for atomic.LoadInt32(&stop) == 0 {
			if rb.Dequeue(&val) {
				atomic.AddUint64(&consumed, 1)
				if val.MachineID > 64 {
					atomic.AddUint64(&corrupted, 1)
				}
				if id%2 == 0 {
					runtime.Gosched()
				}
			}
		}
	}

	producer := func(id int) {
		defer wg.Done()
		for i := 0; i < opsPerProducer; i++ {
			evt := &ContactEvent{
				MachineID: uint16(id % 64),
				BondID:    uint32(i),
				Timestamp: uint64(time.Now().UnixNano()),
				Force_mN:  150.0,
				Status:    1,
			}
			for !rb.Enqueue(evt) {
				runtime.Gosched()
			}
			atomic.AddUint64(&produced, 1)
		}
	}

	for i := 0; i < numConsumers; i++ {
		wg.Add(1)
		go consumer(i)
	}
	for i := 0; i < numProducers; i++ {
		wg.Add(1)
		go producer(i)
	}

	wg.Wait()
	atomic.StoreInt32(&stop, 1)

	for !rb.IsEmpty() {
		var val ContactEvent
		if rb.Dequeue(&val) {
			atomic.AddUint64(&consumed, 1)
			if val.MachineID > 64 {
				atomic.AddUint64(&corrupted, 1)
			}
		}
	}

	t.Logf("Buggy: produced=%d consumed=%d corrupted=%d",
		atomic.LoadUint64(&produced),
		atomic.LoadUint64(&consumed),
		atomic.LoadUint64(&corrupted))

	if corrupted > 0 {
		t.Logf("Confirmed: buggy version exhibits ABA-induced data corruption (%d corrupted entries)", corrupted)
	}
}

func TestVersioned_NoCorruption(t *testing.T) {
	const numProducers = 8
	const numConsumers = 4
	const opsPerProducer = 50000
	const smallCapacity = 8

	rb := NewRingBufferVersioned(smallCapacity)

	var produced uint64
	var consumed uint64
	var corrupted uint64
	var wg sync.WaitGroup
	var stop int32

	consumer := func(id int) {
		defer wg.Done()
		var val ContactEvent
		for atomic.LoadInt32(&stop) == 0 {
			if rb.Dequeue(&val) {
				atomic.AddUint64(&consumed, 1)
				if val.MachineID > 64 {
					atomic.AddUint64(&corrupted, 1)
				}
				if id%2 == 0 {
					runtime.Gosched()
				}
			}
		}
	}

	producer := func(id int) {
		defer wg.Done()
		for i := 0; i < opsPerProducer; i++ {
			evt := &ContactEvent{
				MachineID: uint16(id % 64),
				BondID:    uint32(i),
				Timestamp: uint64(time.Now().UnixNano()),
				Force_mN:  150.0,
				Status:    1,
			}
			for !rb.Enqueue(evt) {
				runtime.Gosched()
			}
			atomic.AddUint64(&produced, 1)
		}
	}

	for i := 0; i < numConsumers; i++ {
		wg.Add(1)
		go consumer(i)
	}
	for i := 0; i < numProducers; i++ {
		wg.Add(1)
		go producer(i)
	}

	wg.Wait()
	atomic.StoreInt32(&stop, 1)

	for !rb.IsEmpty() {
		var val ContactEvent
		if rb.Dequeue(&val) {
			atomic.AddUint64(&consumed, 1)
			if val.MachineID > 64 {
				atomic.AddUint64(&corrupted, 1)
			}
		}
	}

	t.Logf("Versioned: produced=%d consumed=%d corrupted=%d",
		atomic.LoadUint64(&produced),
		atomic.LoadUint64(&consumed),
		atomic.LoadUint64(&corrupted))

	if corrupted > 0 {
		t.Fatalf("Versioned version should have ZERO corruption, but got %d corrupted entries", corrupted)
	}
}

func TestHazard_NoCorruption(t *testing.T) {
	const numProducers = 8
	const numConsumers = 4
	const opsPerProducer = 10000

	rb := NewRingBufferHazard()

	var produced uint64
	var consumed uint64
	var corrupted uint64
	var wg sync.WaitGroup
	var stop int32

	consumer := func(id int) {
		defer wg.Done()
		var val ContactEvent
		for atomic.LoadInt32(&stop) == 0 {
			if rb.Dequeue(&val) {
				atomic.AddUint64(&consumed, 1)
				if val.MachineID > 64 {
					atomic.AddUint64(&corrupted, 1)
				}
				if id%2 == 0 {
					runtime.Gosched()
				}
			}
		}
	}

	producer := func(id int) {
		defer wg.Done()
		for i := 0; i < opsPerProducer; i++ {
			evt := &ContactEvent{
				MachineID: uint16(id % 64),
				BondID:    uint32(i),
				Timestamp: uint64(time.Now().UnixNano()),
				Force_mN:  150.0,
				Status:    1,
			}
			for !rb.Enqueue(evt) {
				runtime.Gosched()
			}
			atomic.AddUint64(&produced, 1)
		}
	}

	for i := 0; i < numConsumers; i++ {
		wg.Add(1)
		go consumer(i)
	}
	for i := 0; i < numProducers; i++ {
		wg.Add(1)
		go producer(i)
	}

	wg.Wait()
	atomic.StoreInt32(&stop, 1)

	for !rb.IsEmpty() {
		var val ContactEvent
		if rb.Dequeue(&val) {
			atomic.AddUint64(&consumed, 1)
			if val.MachineID > 64 {
				atomic.AddUint64(&corrupted, 1)
			}
		}
	}

	t.Logf("Hazard: produced=%d consumed=%d corrupted=%d",
		atomic.LoadUint64(&produced),
		atomic.LoadUint64(&consumed),
		atomic.LoadUint64(&corrupted))

	if corrupted > 0 {
		t.Fatalf("Hazard version should have ZERO corruption, but got %d corrupted entries", corrupted)
	}
}

func TestVersioned_SequencingCorrectness(t *testing.T) {
	rb := NewRingBufferVersioned(16)
	const N = 100000

	var wg sync.WaitGroup
	consumerDone := make(chan struct{})

	var recvOrder []uint32
	var mu sync.Mutex

	wg.Add(1)
	go func() {
		defer wg.Done()
		var val ContactEvent
		for i := 0; i < N; i++ {
			for !rb.Dequeue(&val) {
				runtime.Gosched()
			}
			mu.Lock()
			recvOrder = append(recvOrder, val.BondID)
			mu.Unlock()
		}
	}()

	wg.Add(1)
	go func() {
		defer wg.Done()
		for i := 0; i < N; i++ {
			evt := &ContactEvent{BondID: uint32(i)}
			for !rb.Enqueue(evt) {
				runtime.Gosched()
			}
		}
	}()

	wg.Wait()

	if len(recvOrder) != N {
		t.Fatalf("Expected %d items, got %d", N, len(recvOrder))
	}
	for i, id := range recvOrder {
		if id != uint32(i) {
			t.Fatalf("Sequence mismatch at %d: expected %d, got %d", i, i, id)
		}
	}
	t.Logf("Single-producer single-consumer sequencing: PASS (all %d items in order)", N)
}

func TestVersioned_MultiProducerUnique(t *testing.T) {
	const numProducers = 8
	const opsPerProducer = 10000
	rb := NewRingBufferVersioned(256)

	var wg sync.WaitGroup
	seen := make(map[uint32]int)
	var mu sync.Mutex
	var produced uint64
	var consumed uint64

	wg.Add(1)
	go func() {
		defer wg.Done()
		total := numProducers * opsPerProducer
		var val ContactEvent
		for atomic.LoadUint64(&consumed) < uint64(total) {
			if rb.Dequeue(&val) {
				mu.Lock()
				seen[val.BondID]++
				mu.Unlock()
				atomic.AddUint64(&consumed, 1)
			}
		}
	}()

	for p := 0; p < numProducers; p++ {
		wg.Add(1)
		go func(id int) {
			defer wg.Done()
			for i := 0; i < opsPerProducer; i++ {
				evt := &ContactEvent{
					BondID:    uint32(id*opsPerProducer + i),
					MachineID: uint16(id),
				}
				for !rb.Enqueue(evt) {
					runtime.Gosched()
				}
				atomic.AddUint64(&produced, 1)
			}
		}(p)
	}

	wg.Wait()

	total := numProducers * opsPerProducer
	if len(seen) != total {
		t.Fatalf("Expected %d unique items, got %d", total, len(seen))
	}
	for id, count := range seen {
		if count != 1 {
			t.Fatalf("BondID %d appeared %d times (should appear exactly once)", id, count)
		}
	}
	t.Logf("Multi-producer uniqueness: PASS (%d items, all unique)", total)
}
