.PHONY: build clean flash

build:
	west build -b nrf52833dk_nrf52833

clean:
	rm -r build/*

flash:
	west flash --erase

