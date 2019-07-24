import unittest

from config_gen import gen_local_config


class TestGenLocalConfig(unittest.TestCase):
    def setUp(self) -> None:
        pass

    def test_validate_global_config_empty(self):
        # TODO
        cfg = {}
        self.assertTrue(gen_local_config.validate_global_config(cfg))

    def test_validate_global_config_missing(self):
        # TODO
        cfg = {}
        self.assertFalse(gen_local_config.validate_global_config(cfg))

    def test_gen_local_config(self):
        # TODO
        pass


if __name__ == '__main__':
    unittest.main()

