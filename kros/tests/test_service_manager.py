import threading
import unittest
from dataclasses import dataclass

from kros.service import ServiceManager, ServiceType


@dataclass(frozen=True)
class AddReq:
    a: int
    b: int


@dataclass(frozen=True)
class AddRes:
    sum: int


Add = ServiceType("Add", AddReq, AddRes)
Other = ServiceType("Other", AddReq, AddRes)


class TestServiceManager(unittest.TestCase):
    def setUp(self) -> None:
        self.sm = ServiceManager()
        self.addCleanup(self.sm.shutdown)

    def test_call_returns_response(self) -> None:
        self.sm.create_service("add", Add, lambda r: AddRes(r.a + r.b))
        self.assertEqual(self.sm.call("add", Add, AddReq(1, 2)), AddRes(3))

    def test_call_async_returns_future(self) -> None:
        self.sm.create_service("add", Add, lambda r: AddRes(r.a + r.b))
        self.assertEqual(self.sm.call_async("add", Add, AddReq(2, 2)).result(5), AddRes(4))

    def test_no_server_raises_lookup_error(self) -> None:
        with self.assertRaises(LookupError):
            self.sm.call("none", Add, AddReq(1, 2))

    def test_duplicate_server_raises(self) -> None:
        self.sm.create_service("add", Add, lambda r: AddRes(0))
        with self.assertRaises(ValueError):
            self.sm.create_service("add", Add, lambda r: AddRes(0))

    def test_type_mismatch_raises(self) -> None:
        self.sm.create_service("add", Add, lambda r: AddRes(0))
        with self.assertRaises(TypeError):
            self.sm.call("add", Other, AddReq(1, 2))  # 다른 서비스 타입
        with self.assertRaises(TypeError):
            self.sm.call("add", Add, AddRes(1))  # type: ignore[misc]  # 잘못된 request 타입

    def test_handler_exception_reaches_caller(self) -> None:
        self.sm.create_service("add", Add, lambda r: 1 / 0)  # type: ignore[arg-type,return-value]
        with self.assertRaises(ZeroDivisionError):
            self.sm.call("add", Add, AddReq(1, 2))

    def test_handler_wrong_response_type_raises(self) -> None:
        self.sm.create_service("add", Add, lambda r: "oops")  # type: ignore[arg-type,return-value]
        with self.assertRaises(TypeError):
            self.sm.call("add", Add, AddReq(1, 2))

    def test_requests_handled_in_order_one_at_a_time(self) -> None:
        seen: list[int] = []
        def handler(r: AddReq) -> AddRes:
            seen.append(r.a)
            return AddRes(r.a)

        self.sm.create_service("add", Add, handler)
        futures = [self.sm.call_async("add", Add, AddReq(i, 0)) for i in range(50)]
        for f in futures:
            f.result(5)
        self.assertEqual(seen, list(range(50)))

    def test_destroy_then_call_raises_and_name_is_reusable(self) -> None:
        server = self.sm.create_service("add", Add, lambda r: AddRes(0))
        self.sm.destroy_service(server)
        with self.assertRaises(LookupError):
            self.sm.call("add", Add, AddReq(1, 2))
        self.sm.create_service("add", Add, lambda r: AddRes(7))
        self.assertEqual(self.sm.call("add", Add, AddReq(1, 2)), AddRes(7))

    def test_blocking_self_call_deadlocks_until_timeout(self) -> None:
        def handler(r: AddReq) -> AddRes:
            return self.sm.call("add", Add, r, timeout=0.2)  # 자기 서버 스레드가 점유 중

        self.sm.create_service("add", Add, handler)
        with self.assertRaises(TimeoutError):
            self.sm.call("add", Add, AddReq(1, 2), timeout=5)

    def test_servers_run_in_separate_threads(self) -> None:
        release = threading.Event()
        self.sm.create_service("slow", Add, lambda r: (release.wait(5), AddRes(0))[1])
        self.sm.create_service("fast", Add, lambda r: AddRes(1))
        slow = self.sm.call_async("slow", Add, AddReq(0, 0))
        self.assertEqual(self.sm.call("fast", Add, AddReq(0, 0), timeout=5), AddRes(1))
        release.set()
        slow.result(5)
