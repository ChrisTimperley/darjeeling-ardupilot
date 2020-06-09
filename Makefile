.PHONY: scenarios

scenarios:
	bin/install_scenarios

src/darjeeling_ardupilot/data/mavproxy:
	./build-mavproxy.sh
