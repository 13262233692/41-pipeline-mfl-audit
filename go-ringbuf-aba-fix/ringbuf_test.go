package ringbuf

import (
	"runtime"
	"sync"
	"sync/atomic"
	"testing"
	"time"
)

func BenchmarkVersioned_SPSC(b *testing.B) {
	rb := NewRingBufferVersioned(1024)
	evt := &ContactEvent{MachineID: 1, BondID: 42, Force_mN: 150.0}
	var val ContactEvent

	b.ReportAllocs()
	b.ResetTimer()

	go func() {
		for i := 0; i < b.N; i++ {
			for !rb.Enqueue(evt) {
				runtime.Gosched()
			}
		}
	}()

	for i := 0; i < b.N; i++ {
		for !rb.Dequeue(&val) {
			runtime.Gosched()
		}
	}
}

func BenchmarkVersioned_MPMC_8P8C(b *testing.B) {
	benchmarkMPMC(b, NewRingBufferVersioned(4096), 8, 8)
}

func BenchmarkVersioned_MPMC_64P16C(b *testing.B) {
	benchmarkMPMC(b, NewRingBufferVersioned(8192), 64, 16)
}

func BenchmarkHazard_SPSC(b *testing.B) {
	rb := NewRingBufferHazard()
	evt := &ContactEvent{MachineID: 1, BondID: 42, Force_mN: 150.0}
	var val ContactEvent

	b.ReportAllocs()
	b.ResetTimer()

	go func() {
		for i := 0; i < b.N; i++ {
			for !rb.Enqueue(evt) {
				runtime.Gosched()
			}
		}
	}()

	for i := 0; i < b.N; i++ {
		for !rb.Dequeue(&val) {
			runtime.Gosched()
		}
	}
}

func BenchmarkHazard_MPMC_8P8C(b *testing.B) {
	benchmarkMPMC_Hazard(b, 8, 8)
}

func BenchmarkHazard_MPMC_64P16C(b *testing.B) {
	benchmarkMPMC_Hazard(b, 64, 16)
}

func BenchmarkBuggy_SPSC(b *testing.B) {
	rb := NewRingBufferBuggy(1024)
	evt := &ContactEvent{MachineID: 1, BondID: 42, Force_mN: 150.0}
	var val ContactEvent

	b.ReportAllocs()
	b.ResetTimer()

	go func() {
		for i := 0; i < b.N; i++ {
			for !rb.Enqueue(evt) {
				runtime.Gosched()
			}
		}
	}()

	for i := 0; i < b.N; i++ {
		for !rb.Dequeue(&val) {
			runtime.Gosched()
		}
	}
}

func benchmarkMPMC(b *testing.B, rb *RingBufferVersioned, numProducers, numConsumers int) {
	ops := b.N / numProducers
	if ops < 1 {
		ops = 1
	}

	b.ReportAllocs()
	b.ResetTimer()

	var wg sync.WaitGroup
	var produced uint64
	var consumed uint64

	for p := 0; p < numProducers; p++ {
		wg.Add(1)
		go func(id int) {
			defer wg.Done()
			evt := &ContactEvent{MachineID: uint16(id % 64)}
			for i := 0; i < ops; i++ {
				evt.BondID = uint32(i)
				evt.Timestamp = uint64(time.Now().UnixNano())
				for !rb.Enqueue(evt) {
					runtime.Gosched()
				}
				atomic.AddUint64(&produced, 1)
			}
		}(p)
	}

	total := uint64(ops * numProducers)
	for c := 0; c < numConsumers; c++ {
		wg.Add(1)
		go func() {
			defer wg.Done()
			var val ContactEvent
			for atomic.LoadUint64(&consumed) < total {
				if rb.Dequeue(&val) {
					atomic.AddUint64(&consumed, 1)
				}
			}
		}()
	}

	wg.Wait()
}

func benchmarkMPMC_Hazard(b *testing.B, numProducers, numConsumers int) {
	rb := NewRingBufferHazard()
	ops := b.N / numProducers
	if ops < 1 {
		ops = 1
	}

	b.ReportAllocs()
	b.ResetTimer()

	var wg sync.WaitGroup
	var produced uint64
	var consumed uint64

	for p := 0; p < numProducers; p++ {
		wg.Add(1)
		go func(id int) {
			defer wg.Done()
			evt := &ContactEvent{MachineID: uint16(id % 64)}
			for i := 0; i < ops; i++ {
				evt.BondID = uint32(i)
				evt.Timestamp = uint64(time.Now().UnixNano())
				for !rb.Enqueue(evt) {
					runtime.Gosched()
				}
				atomic.AddUint64(&produced, 1)
			}
		}(p)
	}

	total := uint64(ops * numProducers)
	for c := 0; c < numConsumers; c++ {
		wg.Add(1)
		go func() {
			defer wg.Done()
			var val ContactEvent
			for atomic.LoadUint64(&consumed) < total {
				if rb.Dequeue(&val) {
					atomic.AddUint64(&consumed, 1)
				}
			}
		}()
	}

	wg.Wait()
}

func TestStress_64BondingMachines(b *testing.T) {
	const numMachines = 64
	const numConsumers = 16
	const duration = 5 * time.Second

	rb := NewRingBufferVersioned(32768)
	var wg sync.WaitGroup
	var stop int32

	var totalEnqueued uint64
	var totalDequeued uint64
	var corrupted uint64

	for m := 0; m < numMachines; m++ {
		wg.Add(1)
		go func(id int) {
			defer wg.Done()
			evt := &ContactEvent{MachineID: uint16(id)}
			localCount := uint64(0)
			for atomic.LoadInt32(&stop) == 0 {
				evt.BondID = uint32(localCount)
				evt.Timestamp = uint64(time.Now().UnixNano())
				evt.Force_mN = 100.0 + float32(id%10)*10.0
				evt.Status = 1

				if rb.Enqueue(evt) {
					localCount++
				}
			}
			atomic.AddUint64(&totalEnqueued, localCount)
		}(m)
	}

	for c := 0; c < numConsumers; c++ {
		wg.Add(1)
		go func() {
			defer wg.Done()
			var val ContactEvent
			localCount := uint64(0)
			localCorrupt := uint64(0)
			for atomic.LoadInt32(&stop) == 0 {
				if rb.Dequeue(&val) {
					if val.MachineID > 64 {
						localCorrupt++
					}
					if val.Status != 1 {
						localCorrupt++
					}
					localCount++
				}
			}
			atomic.AddUint64(&totalDequeued, localCount)
			atomic.AddUint64(&corrupted, localCorrupt)
		}()
	}

	time.Sleep(duration)
	atomic.StoreInt32(&stop, 1)
	wg.Wait()

	drainStart := time.Now()
	for !rb.IsEmpty() && time.Since(drainStart) < 2*time.Second {
		var val ContactEvent
		if rb.Dequeue(&val) {
			atomic.AddUint64(&totalDequeued, 1)
			if val.MachineID > 64 || val.Status != 1 {
				atomic.AddUint64(&corrupted, 1)
			}
		}
	}

	b.Logf("==== 64 Bonding Machine Stress Test (5 sec) ====")
	b.Logf("  Enqueued:   %d (%.0f ops/sec)",
		atomic.LoadUint64(&totalEnqueued),
		float64(atomic.LoadUint64(&totalEnqueued))/duration.Seconds())
	b.Logf("  Dequeued:   %d (%.0f ops/sec)",
		atomic.LoadUint64(&totalDequeued),
		float64(atomic.LoadUint64(&totalDequeued))/duration.Seconds())
	b.Logf("  Corrupted:  %d", atomic.LoadUint64(&corrupted))
	b.Logf("  Loss rate:  %.6f%%",
		float64(atomic.LoadUint64(&totalEnqueued)-atomic.LoadUint64(&totalDequeued))/
			float64(atomic.LoadUint64(&totalEnqueued))*100)

	if corrupted > 0 {
		b.Fatalf("FATAL: %d corrupted events detected during stress test", corrupted)
	}
	b.Logf("RESULT: PASS — Zero data corruption, no segfault")
}

func TestThroughput_Comparison(b *testing.T) {
	const numMachines = 64
	const numConsumers = 16
	const duration = 3 * time.Second

	testVersioned := func() (ops uint64, corrupt uint64) {
		rb := NewRingBufferVersioned(32768)
		var wg sync.WaitGroup
		var stop int32
		var total uint64
		var corruptions uint64

		for m := 0; m < numMachines; m++ {
			wg.Add(1)
			go func(id int) {
				defer wg.Done()
				evt := &ContactEvent{MachineID: uint16(id)}
				for atomic.LoadInt32(&stop) == 0 {
					if rb.Enqueue(evt) {
						atomic.AddUint64(&total, 1)
					}
				}
			}(m)
		}
		for c := 0; c < numConsumers; c++ {
			wg.Add(1)
			go func() {
				defer wg.Done()
				var val ContactEvent
				for atomic.LoadInt32(&stop) == 0 {
					if rb.Dequeue(&val) {
						if val.MachineID > 64 {
							atomic.AddUint64(&corruptions, 1)
						}
					}
				}
			}()
		}

		time.Sleep(duration)
		atomic.StoreInt32(&stop, 1)
		wg.Wait()
		return total, corruptions
	}

	testHazard := func() (ops uint64, corrupt uint64) {
		rb := NewRingBufferHazard()
		var wg sync.WaitGroup
		var stop int32
		var total uint64
		var corruptions uint64

		for m := 0; m < numMachines; m++ {
			wg.Add(1)
			go func(id int) {
				defer wg.Done()
				evt := &ContactEvent{MachineID: uint16(id)}
				for atomic.LoadInt32(&stop) == 0 {
					if rb.Enqueue(evt) {
						atomic.AddUint64(&total, 1)
					}
				}
			}(m)
		}
		for c := 0; c < numConsumers; c++ {
			wg.Add(1)
			go func() {
				defer wg.Done()
				var val ContactEvent
				for atomic.LoadInt32(&stop) == 0 {
					if rb.Dequeue(&val) {
						if val.MachineID > 64 {
							atomic.AddUint64(&corruptions, 1)
						}
					}
				}
			}()
		}

		time.Sleep(duration)
		atomic.StoreInt32(&stop, 1)
		wg.Wait()
		return total, corruptions
	}

	vOps, vCorrupt := testVersioned()
	hOps, hCorrupt := testHazard()

	b.Logf("==== Throughput Comparison (%d machines, %d consumers, %v) ====",
		numMachines, numConsumers, duration)
	b.Logf("  Versioned CAS: %.0f ops/sec, corruption=%d",
		float64(vOps)/duration.Seconds(), vCorrupt)
	b.Logf("  Hazard Ptrs:   %.0f ops/sec, corruption=%d",
		float64(hOps)/duration.Seconds(), hCorrupt)
	b.Logf("  Versioned is %.1fx faster than Hazard",
		float64(vOps)/float64(hOps))

	if vCorrupt > 0 || hCorrupt > 0 {
		b.Fatalf("Corruption detected! versioned=%d, hazard=%d", vCorrupt, hCorrupt)
	}
}

func TestBasicOperations(t *testing.T) {
	t.Run("Versioned", func(t *testing.T) {
		rb := NewRingBufferVersioned(8)
		if !rb.IsEmpty() {
			t.Fatal("New buffer should be empty")
		}
		if rb.Capacity() != 8 {
			t.Fatalf("Expected capacity 8, got %d", rb.Capacity())
		}

		for i := 0; i < 8; i++ {
			ok := rb.Enqueue(&ContactEvent{BondID: uint32(i)})
			if !ok {
				t.Fatalf("Enqueue %d failed", i)
			}
		}
		if rb.Size() != 8 {
			t.Fatalf("Expected size 8, got %d", rb.Size())
		}

		ok := rb.Enqueue(&ContactEvent{BondID: 99})
		if ok {
			t.Fatal("Enqueue to full buffer should fail")
		}

		for i := 0; i < 8; i++ {
			var val ContactEvent
			ok := rb.Dequeue(&val)
			if !ok {
				t.Fatalf("Dequeue %d failed", i)
			}
			if val.BondID != uint32(i) {
				t.Fatalf("Expected BondID %d, got %d", i, val.BondID)
			}
		}
		if !rb.IsEmpty() {
			t.Fatal("Buffer should be empty after dequeuing all")
		}
	})

	t.Run("Buggy", func(t *testing.T) {
		rb := NewRingBufferBuggy(8)
		if !rb.IsEmpty() {
			t.Fatal("New buffer should be empty")
		}

		for i := 0; i < 8; i++ {
			ok := rb.Enqueue(&ContactEvent{BondID: uint32(i)})
			if !ok {
				t.Fatalf("Enqueue %d failed", i)
			}
		}

		for i := 0; i < 8; i++ {
			var val ContactEvent
			ok := rb.Dequeue(&val)
			if !ok {
				t.Fatalf("Dequeue %d failed", i)
			}
			if val.BondID != uint32(i) {
				t.Fatalf("Expected BondID %d, got %d", i, val.BondID)
			}
		}
	})

	t.Run("Hazard", func(t *testing.T) {
		rb := NewRingBufferHazard()
		if !rb.IsEmpty() {
			t.Fatal("New buffer should be empty")
		}

		for i := 0; i < 10; i++ {
			ok := rb.Enqueue(&ContactEvent{BondID: uint32(i)})
			if !ok {
				t.Fatalf("Enqueue %d failed", i)
			}
		}

		for i := 0; i < 10; i++ {
			var val ContactEvent
			ok := rb.Dequeue(&val)
			if !ok {
				t.Fatalf("Dequeue %d failed", i)
			}
			if val.BondID != uint32(i) {
				t.Fatalf("Expected BondID %d, got %d", i, val.BondID)
			}
		}
		if !rb.IsEmpty() {
			t.Fatal("Buffer should be empty after dequeuing all")
		}
	})
}
