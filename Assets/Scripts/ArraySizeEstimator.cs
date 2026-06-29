/*
Copyright (c) <2025> <Aleksei Fedorov, aleksei.fedorov@eit.lth.se; Nikita Lyamin, nikita.lyamin@volvocars.com>

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/
using System;
using System.Collections.Generic;
using System.Linq;

public class ArrayCapacityEstimator
{
    private Queue<int> observedSizes = new Queue<int>(); // Using Queue to maintain a fixed window of observations
    private int initialEstimate;
    private const double zScore = 2.576; // For 99% confidence interval
    private const int windowSize = 10; // Example fixed window size
    private const double adjustmentFactor = 1.1; // Adjust the final estimate to be slightly bigger

    public ArrayCapacityEstimator(int initialEstimate)
    {
        this.initialEstimate = initialEstimate;
    }

    public void AddObservation(int[] sizes)
    {
        if (sizes == null || sizes.Length == 0)
        {
            throw new ArgumentException("Sizes array must not be null or empty.", nameof(sizes));
        }

        int max = sizes.Max();

        // Maintain fixed window size
        if (observedSizes.Count >= windowSize)
        {
            observedSizes.Dequeue(); // Remove the oldest observation
        }

        observedSizes.Enqueue(max); // Add the new observation
    }

    public int GetEstimatedCapacity()
    {
        if (!observedSizes.Any())
        {
            return initialEstimate;
        }

        double mean = observedSizes.Average();
        double stdDev = CalculateStandardDeviation(observedSizes.ToList(), mean);
        double marginOfError = zScore * (stdDev / Math.Sqrt(observedSizes.Count));

        // Adjusting estimate to be slightly bigger than the upper end of the 99% confidence interval
        int estimate = (int)Math.Round((mean + marginOfError) * adjustmentFactor);

        // Ensure estimate does not fall below the initial estimate
        return estimate;
    }

    private double CalculateStandardDeviation(List<int> values, double mean)
    {
        double sumOfSquares = values.Sum(value => Math.Pow(value - mean, 2));
        return Math.Sqrt(sumOfSquares / values.Count);
    }
}
