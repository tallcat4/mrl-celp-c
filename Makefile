# mrl-celp-c: デコーダとエンコーダを作ってテストする。   make; make test
all:
	$(MAKE) -C decoder
	$(MAKE) -C encoder

test: all
	$(MAKE) -C decoder test
	$(MAKE) -C encoder test

clean:
	$(MAKE) -C decoder clean
	$(MAKE) -C encoder clean

.PHONY: all test clean
