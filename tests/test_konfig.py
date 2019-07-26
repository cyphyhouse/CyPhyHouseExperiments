import unittest

from cph import konfig


class TestGenLocalConfig(unittest.TestCase):
    def setUp(self) -> None:
        pass

    def test_validate_global_config_empty(self):
        # TODO
        cfg = {}
        self.assertTrue(konfig.validate_global_config(cfg))

    def test_validate_global_config_missing(self):
        # TODO
        cfg = {}
        self.assertFalse(konfig.validate_global_config(cfg))

    def test_gen_local_config(self):
        # TODO
        pass


if __name__ == '__main__':
    unittest.main()

