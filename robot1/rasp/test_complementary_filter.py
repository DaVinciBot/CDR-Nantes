#!/usr/bin/env python3
"""
Test du filtre complémentaire adaptatif : fusion LiDAR + Teensy.

Scenario:
1. Teensy derive (erreur odométrie accumulée)
2. LiDAR fournit correction via SVD avec confiance variable
3. Alpha adaptatif ajuste le blend selon confiance
4. Résultat : position fusionnée stable
"""

import math
import sys
sys.path.insert(0, '/path/to/robot1/rasp')

from robot import Robot


def test_complementary_filter_low_confidence():
    """
    Test 1 : Confiance LiDAR basse → trust Teensy 85%
    """
    print("\n=== TEST 1: Low LiDAR Confidence ===")
    robot = Robot("BLEU")
    
    # Scenario : Teensy dit (1000, 2000), LiDAR corrige à (995, 1990) avec conf=0.2
    lidar_x = 995.0
    lidar_y = 1990.0
    teensy_x = 1000.0
    teensy_y = 2000.0
    confidence = 0.15
    
    x_fused, y_fused, alpha = robot._apply_complementary_filter(
        lidar_x, lidar_y, teensy_x, teensy_y, confidence
    )
    
    print(f"  Input: LiDAR=({lidar_x}, {lidar_y}), Teensy=({teensy_x}, {teensy_y})")
    print(f"  Confidence: {confidence:.2f} → alpha={alpha:.2f} (trust 85% Teensy)")
    print(f"  Output: ({x_fused:.1f}, {y_fused:.1f})")
    
    # Vérification : alpha ~0.85, donc résultat proche de Teensy
    expected_x = (1.0 - 0.85) * lidar_x + 0.85 * teensy_x
    expected_y = (1.0 - 0.85) * lidar_y + 0.85 * teensy_y
    
    assert abs(x_fused - expected_x) < 0.1, f"X mismatch: {x_fused} vs {expected_x}"
    assert abs(y_fused - expected_y) < 0.1, f"Y mismatch: {y_fused} vs {expected_y}"
    assert alpha > 0.80 and alpha <= 0.85, f"Alpha out of range: {alpha}"
    
    print("  ✓ PASSED")


def test_complementary_filter_high_confidence():
    """
    Test 2 : Confiance LiDAR haute → trust LiDAR 75%
    """
    print("\n=== TEST 2: High LiDAR Confidence ===")
    robot = Robot("BLEU")
    
    # Scenario : Teensy a dérived à (1020, 2015), LiDAR corrige à (1010, 2005) avec conf=0.9
    lidar_x = 1010.0
    lidar_y = 2005.0
    teensy_x = 1020.0
    teensy_y = 2015.0
    confidence = 0.90
    
    x_fused, y_fused, alpha = robot._apply_complementary_filter(
        lidar_x, lidar_y, teensy_x, teensy_y, confidence
    )
    
    print(f"  Input: LiDAR=({lidar_x}, {lidar_y}), Teensy=({teensy_x}, {teensy_y})")
    print(f"  Confidence: {confidence:.2f} → alpha={alpha:.2f} (trust 75% LiDAR)")
    print(f"  Output: ({x_fused:.1f}, {y_fused:.1f})")
    
    # Vérification : alpha ~0.25, donc résultat proche de LiDAR
    expected_x = (1.0 - 0.25) * lidar_x + 0.25 * teensy_x
    expected_y = (1.0 - 0.25) * lidar_y + 0.25 * teensy_y
    
    assert abs(x_fused - expected_x) < 0.1, f"X mismatch: {x_fused} vs {expected_x}"
    assert abs(y_fused - expected_y) < 0.1, f"Y mismatch: {y_fused} vs {expected_y}"
    assert alpha >= 0.20 and alpha < 0.30, f"Alpha out of range: {alpha}"
    
    print("  ✓ PASSED")


def test_complementary_filter_medium_confidence():
    """
    Test 3 : Confiance moyenne → blend équilibré
    """
    print("\n=== TEST 3: Medium LiDAR Confidence ===")
    robot = Robot("BLEU")
    
    # Scenario : Teensy à (1000, 2000), LiDAR corrige à (1005, 2010) avec conf=0.5
    lidar_x = 1005.0
    lidar_y = 2010.0
    teensy_x = 1000.0
    teensy_y = 2000.0
    confidence = 0.50
    
    x_fused, y_fused, alpha = robot._apply_complementary_filter(
        lidar_x, lidar_y, teensy_x, teensy_y, confidence
    )
    
    print(f"  Input: LiDAR=({lidar_x}, {lidar_y}), Teensy=({teensy_x}, {teensy_y})")
    print(f"  Confidence: {confidence:.2f} → alpha={alpha:.2f}")
    print(f"  Output: ({x_fused:.1f}, {y_fused:.1f})")
    
    # Vérification : alpha entre 0.25 et 0.85
    assert 0.25 <= alpha <= 0.85, f"Alpha out of range: {alpha}"
    
    # Résultat intermédiaire
    expected_x = (1.0 - alpha) * lidar_x + alpha * teensy_x
    expected_y = (1.0 - alpha) * lidar_y + alpha * teensy_y
    
    assert abs(x_fused - expected_x) < 0.1, f"X mismatch: {x_fused} vs {expected_x}"
    assert abs(y_fused - expected_y) < 0.1, f"Y mismatch: {y_fused} vs {expected_y}"
    
    print("  ✓ PASSED")


def test_complementary_filter_edge_cases():
    """
    Test 4 : Cas limites
    """
    print("\n=== TEST 4: Edge Cases ===")
    robot = Robot("BLEU")
    
    # Case A : Confiance = 0.0 (pas d'info LiDAR)
    x_fused, y_fused, alpha = robot._apply_complementary_filter(
        500.0, 600.0, 1000.0, 2000.0, confidence=0.0
    )
    print(f"  Case A (conf=0.0): alpha={alpha:.2f} (should be 0.85)")
    assert alpha == 0.85, f"Alpha should be 0.85 for conf<0.2, got {alpha}"
    
    # Case B : Confiance = 1.0 (parfait LiDAR)
    x_fused, y_fused, alpha = robot._apply_complementary_filter(
        500.0, 600.0, 1000.0, 2000.0, confidence=1.0
    )
    print(f"  Case B (conf=1.0): alpha={alpha:.2f} (should be 0.25)")
    assert alpha == 0.25, f"Alpha should be 0.25 for conf>0.8, got {alpha}"
    
    # Case C : Transition boundary à 0.2
    x_fused, y_fused, alpha_02 = robot._apply_complementary_filter(
        500.0, 600.0, 1000.0, 2000.0, confidence=0.2
    )
    x_fused, y_fused, alpha_21 = robot._apply_complementary_filter(
        500.0, 600.0, 1000.0, 2000.0, confidence=0.21
    )
    print(f"  Case C: alpha(0.2)={alpha_02:.2f}, alpha(0.21)={alpha_21:.2f}")
    assert alpha_21 < alpha_02, "Alpha should decrease as confidence increases"
    
    print("  ✓ PASSED")


def test_alpha_progression():
    """
    Test 5 : Vérifier la progression linéaire d'alpha entre 0.2 et 0.8
    """
    print("\n=== TEST 5: Alpha Progression ===")
    robot = Robot("BLEU")
    
    confidences = [0.0, 0.2, 0.35, 0.5, 0.65, 0.8, 1.0]
    alphas = []
    
    for conf in confidences:
        _, _, alpha = robot._apply_complementary_filter(
            500.0, 600.0, 1000.0, 2000.0, confidence=conf
        )
        alphas.append(alpha)
        print(f"  Confidence={conf:.2f} → Alpha={alpha:.3f}")
    
    # Vérifier que alpha décroît (confiance augmente → moins de poids Teensy)
    for i in range(len(alphas) - 1):
        assert alphas[i] >= alphas[i+1], \
            f"Alpha not monotonic: {alphas[i]} > {alphas[i+1]} at indices {i}, {i+1}"
    
    print("  ✓ PASSED - Alpha correctly decreases with confidence")


if __name__ == "__main__":
    print("╔═══════════════════════════════════════════════════════╗")
    print("║  TEST FILTRE COMPLÉMENTAIRE ADAPTATIF - LIDAR/TEENSY  ║")
    print("╚═══════════════════════════════════════════════════════╝")
    
    try:
        test_complementary_filter_low_confidence()
        test_complementary_filter_high_confidence()
        test_complementary_filter_medium_confidence()
        test_complementary_filter_edge_cases()
        test_alpha_progression()
        
        print("\n" + "="*55)
        print("✓ ALL TESTS PASSED")
        print("="*55)
        
    except AssertionError as e:
        print(f"\n✗ TEST FAILED: {e}")
        sys.exit(1)
    except Exception as e:
        print(f"\n✗ ERROR: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
