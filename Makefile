# mrl-celp-c: デコーダ・エンコーダ・誤り訂正を作ってテストする。   make; make test
all:
	$(MAKE) -C decoder
	$(MAKE) -C encoder
	$(MAKE) -C fec

test: all
	$(MAKE) -C decoder test
	$(MAKE) -C encoder test
	$(MAKE) -C fec test

clean:
	$(MAKE) -C decoder clean
	$(MAKE) -C encoder clean
	$(MAKE) -C fec clean

.PHONY: all test clean
