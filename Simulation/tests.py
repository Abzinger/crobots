import unittest
import json
import gridTools.fileReader as fr

class TestRouteList(unittest.TestCase):

    def test_is_list(self):
        self.assertIsInstance(json.loads(fr.get_json_route_list()), list)

    def test_holds_nine_scenarios(self):
        self.assertEqual(len(json.loads(fr.get_json_route_list())), 9)

    def test_first_name_Scenario_1(self):
        self.assertEqual(json.loads(fr.get_json_route_list())[0]["name"], "Scenario 1")

    def test_last_name_not_Scenario_1(self):
        self.assertNotEqual(json.loads(fr.get_json_route_list())[-1]["name"], "Scenario 1")

class TestGetInstructions(unittest.TestCase):

    def test_is_list(self):
        self.assertIsInstance(json.loads(fr.get_instructions(fr.get_route(fr.get_route_list()[0]))), list)

    def test_Scenario_1_has_200_instructions(self):
        self.assertEqual(len(json.loads(fr.get_instructions(fr.get_route(fr.get_route_list()[0])))), 200)

    def test_Scenario_1_third_array_has_2_objects(self):
        self.assertEqual(len(json.loads(fr.get_instructions(fr.get_route(fr.get_route_list()[0])))[2]), 2)

    def test_Scenario_1_fourth_array_has_not_1_object(self):
        self.assertNotEqual(len(json.loads(fr.get_instructions(fr.get_route(fr.get_route_list()[0])))[3]), 1)

class TestGetRealisticInstructions(unittest.TestCase):

    def test_is_list(self):
        self.assertIsInstance(json.loads(fr.get_realistic_instructions(fr.get_route(fr.get_route_list()[0]))), list)

    def test_Scenario_1_has_201_instructions(self):
        self.assertEqual(len(json.loads(fr.get_realistic_instructions(fr.get_route(fr.get_route_list()[0])))), 201)

    def test_Scenario_1_third_array_has_2_objects(self):
        self.assertEqual(len(json.loads(fr.get_realistic_instructions(fr.get_route(fr.get_route_list()[0])))[2]), 2)

    def test_Scenario_1_fourth_array_has_not_1_object(self):
        self.assertNotEqual(len(json.loads(fr.get_realistic_instructions(fr.get_route(fr.get_route_list()[0])))[3]), 1)

class TestGetParkingLayout(unittest.TestCase):

    def test_is_dictionary(self):
        self.assertIsInstance(json.loads(fr.get_parking_layout(fr.get_route(fr.get_route_list()[0]))), dict)

    def test_Scenario_1_width_is_10(self):
        self.assertEqual(json.loads(fr.get_parking_layout(fr.get_route(fr.get_route_list()[0])))["width"], 10)

    def test_Scenario_1_height_is_not_2(self):
        self.assertNotEqual(json.loads(fr.get_parking_layout(fr.get_route(fr.get_route_list()[0])))["height"], 2)

    def test_scenario_6_has_11_machines(self):
        self.assertEqual(len(json.loads(fr.get_parking_layout(fr.get_route(fr.get_route_list()[5])))["machines"]), 11)

    def test_scenario_7_has_10_machines_get_last_step(self):
        self.assertEqual(len(json.loads(fr.get_parking_layout(fr.get_route(fr.get_route_list()[6])), False)["machines"]), 11)

    def test_scenario_8_has_not_10_machines(self):
        self.assertNotEqual(len(json.loads(fr.get_parking_layout(fr.get_route(fr.get_route_list()[5])))["machines"]), 10)


if __name__ == '__main__':
    unittest.main()





