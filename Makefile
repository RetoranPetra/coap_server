.PHONY: build clean

build:
	west build -b nrf52833dk_nrf52833

clean:
	rm -r build/*

