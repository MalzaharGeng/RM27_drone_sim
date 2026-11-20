import pathlib
import tempfile
import unittest
from navigation_accuracy import summarize


class AccuracyTest(unittest.TestCase):
    def report(self, text):
        with tempfile.TemporaryDirectory() as directory:
            path = pathlib.Path(directory)/"paired.csv"
            path.write_text(text)
            return summarize(path)

    def test_bias_and_true_target_error_are_not_fitted_away(self):
        p = self.report("t_s,truth_t_s,vio_n,vio_e,vio_d,truth_n,truth_e,truth_d,target_n,target_e,target_d\n"
                        "0,0,0.2,0,0,0,0,0,10,0,0\n"
                        "1,1,10.2,0,0,10,0,0,10,0,0\n")
        self.assertAlmostEqual(p["error_rmse_m"], .2)
        self.assertAlmostEqual(p["endpoint_true_target_error_m"], 0.)
        self.assertAlmostEqual(p["segments"][0]["start_to_end_error_change_m"], 0.)
        self.assertEqual(len(p["unobserved_intervals"]), 1)

    def test_localization_and_control_error_are_separate(self):
        p = self.report("t_s,truth_t_s,vio_n,vio_e,vio_d,truth_n,truth_e,truth_d,target_n,target_e,target_d\n"
                        "0,0,0,0,-1,0,0,-1,10,0,-1\n"
                        "0.1,0.1,10,0,-1,9.7,0,-1,10,0,-1\n")
        self.assertAlmostEqual(p["endpoint_localization_error_m"], .3)
        self.assertAlmostEqual(p["endpoint_true_target_error_m"], .3)
        self.assertAlmostEqual(p["endpoint_estimated_target_error_m"], 0.)

    def test_relocation_jump_and_invalid_pairing(self):
        p = self.report("t_s,truth_t_s,vio_n,vio_e,vio_d,truth_n,truth_e,truth_d,valid,reference_epoch\n"
                        "0,0,0,0,0,0,0,0,1,0\n"
                        "0.1,0.1,1,0,0,0,0,0,1,1\n"
                        "0.2,0.2,nan,0,0,0,0,0,0,1\n"
                        "0.3,0,1,0,0,0,0,0,1,1\n")
        self.assertEqual(p["reference_changes"], 1)
        self.assertEqual(p["invalid_samples"], 1)
        self.assertEqual(p["unsynchronized_samples"], 1)
        self.assertEqual(len(p["segments"]), 2)
        self.assertAlmostEqual(p["discontinuity_events"][0]["error_step_m"], 1.)

    def test_truth_is_required(self):
        with self.assertRaisesRegex(ValueError, "Independent truth required"):
            self.report("t_s,vio_n,vio_e,vio_d\n0,0,0,0\n")

    def test_rejects_nonfinite_and_nonmonotonic_data(self):
        header = "t_s,truth_t_s,vio_n,vio_e,vio_d,truth_n,truth_e,truth_d\n"
        for body in ("0,0,nan,0,0,0,0,0\n", "0,0,0,0,0,0,0,0\n0,0,0,0,0,0,0,0\n"):
            with self.assertRaises(ValueError):
                self.report(header+body)


if __name__ == "__main__":
    unittest.main()
