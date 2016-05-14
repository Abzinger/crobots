import unittest
import json
import gridTools.fileReader as fr

class TestRouteList(unittest.TestCase):



    def test_is_array(self):
        self.assertIsInstance(json.loads(fr.get_json_route_list()), list)

    def test_holds_nine_scenarios(self):
        self.assertEqual(len(json.loads(fr.get_json_route_list())), 9)

    def test_first_name_Scenario_1(self):
        self.assertEqual(json.loads(fr.get_json_route_list())[0]["name"], "Scenario 1")

if __name__ == '__main__':
    unittest.main()




