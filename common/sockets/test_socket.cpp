/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#include "server.hpp"
#include "client.hpp"
#include <thread>
#include <future>
#include <cassert>
#include <iostream>

static bool test_udp();
static bool test_loopback_data();


static void run_udp_server(std::promise<bool>& ret, const std::string& localIP, const uint16_t port);
static void run_tcp_server(std::promise<bool>& ret, const std::string& localIP, const uint16_t port);
static void run_udp_client(std::promise<bool>& ret, const std::string& remoteIP, const uint16_t port);
static void run_tcp_client(std::promise<bool>& ret, const std::string& remoteIP, const uint16_t port);

int main(int, char**) {
	bool result = true;
	//result &= test_udp();
	//exit(EXIT_FAILURE);
	result &= test_loopback_data();
	exit(result ? EXIT_SUCCESS : EXIT_FAILURE);
}

bool test_udp() {
	std::promise<bool> a;
	//run_udp_client(a, "127.0.0.1", 12345);
	run_udp_server(a, "127.0.0.1", 12345);
}

bool test_loopback_data() {
	std::string localIP = "127.0.0.1";
	std::string remoteIP = "127.0.0.1";
	uint16_t tcpPort = 12345;
	uint16_t udpPort = 12345;
	auto timeout = std::chrono::seconds(1);
	bool result = true;

	std::promise<bool> tsrp;			//!< tcp server result promise
	std::promise<bool> tcrp;			//!< tcp client result promise
	auto tsrf = tsrp.get_future();		//!< tcp server result future
	auto tcrf = tcrp.get_future();		//!< tcp client result future
	std::promise<bool> usrp;			//!< udp server result promise
	std::promise<bool> ucrp;			//!< udp client result promise
	auto usrf = usrp.get_future();		//!< udp server result future
	auto ucrf = ucrp.get_future();		//!< udp client result future

	std::thread tst(run_tcp_server, std::ref(tsrp), std::ref(localIP), tcpPort);
	std::thread tct(run_tcp_client, std::ref(tcrp), std::ref(remoteIP), tcpPort);
	std::thread ust(run_udp_server, std::ref(usrp), std::ref(localIP), udpPort);
	std::thread uct(run_udp_client, std::ref(ucrp), std::ref(remoteIP), udpPort);

	try { result = tsrf.wait_for(timeout) == std::future_status::ready && tsrf.get(); }
	catch (std::exception& e) { std::cout << "TCP server " << e.what() << std::endl; result = false; }
	try { result = tcrf.wait_for(timeout) == std::future_status::ready && tcrf.get(); }
	catch (std::exception& e) { std::cout << "TCP client " << e.what() << std::endl; result = false; }
	try { result = usrf.wait_for(timeout) == std::future_status::ready && usrf.get(); }
	catch (std::exception& e) { std::cout << "UDP server " << e.what() << std::endl; result = false; }
	try { result = ucrf.wait_for(timeout) == std::future_status::ready && ucrf.get(); }
	catch (std::exception& e) { std::cout << "UDP client " << e.what() << std::endl; result = false; }

	tst.join();
	tct.join();
	ust.join();
	uct.join();
	return result;
}


void run_udp_server(std::promise<bool>& ret, const std::string& localIP, const uint16_t port) {
	try {
		UDPServer udpServer(localIP, port, true);
		auto rdata = udpServer.recvfrom();
		udpServer.sendto(rdata);
		ret.set_value(true);
	} catch (...) {
		try {
			ret.set_exception(std::current_exception());
		} catch (...) {}
	}
}

void run_tcp_server(std::promise<bool>& ret, const std::string& localIP, const uint16_t port) {
	try {
		TCPServer tcpServer(localIP, port, true);
		auto conn = tcpServer.await();
		try {
			while (true) {
				// Just send once, so we don't block with the recv call
				auto rdata = conn.recv();
				conn.send(rdata);
			}
		} catch (SocketErrors::DisconnectedError&) {
			// Remote closed, safe to ignore
		}
		ret.set_value(true);
	} catch (...) {
		try {
			ret.set_exception(std::current_exception());
		} catch (...) {}
	}
}

void run_udp_client(
		std::promise<bool>& ret,
		const std::string& remoteIP,
		const uint16_t port) {
	try {
		UDPClient udpClient(remoteIP, port, true);
		std::string testString = "testing some UDP data";
		auto testData = std::vector<char>(testString.begin(), testString.end());
		udpClient.send(testData);
		auto rdata = udpClient.recv();
		ret.set_value(testData == rdata);
		udpClient.close();
	} catch (...) {
		try {
			ret.set_exception(std::current_exception());
		} catch (...) {}
	}
}

void run_tcp_client(
		std::promise<bool>& ret,
		const std::string& remoteIP,
		const uint16_t port) {
	try {
		TCPClient tcpClient(remoteIP, port, true);
		std::string testString = "testing some TCP data";
		auto testData = std::vector<char>(testString.begin(), testString.end());
		tcpClient.send(testData);
		auto rdata = tcpClient.recv();
		ret.set_value(testData == rdata);
		tcpClient.close();
	} catch (...) {
		try {
			ret.set_exception(std::current_exception());
		} catch (...) {}
	}
}
