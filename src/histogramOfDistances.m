clear all;
close all;

% run the whole process three times and plot side by side
N_tries = 3;    
N_pts = 100;
N_bins = 3;
max_dist = 2.0;

figure;
for k=1:N_tries
    % sample points on surface of sphere
    pts = rand(3, N_pts)*2.0 - 1.0;
    pts = normc(pts);

    subplot(N_tries, 2, (k-1)*2 + 1);
    scatter3(pts(1, :), pts(2, :), pts(3, :));

    % for one of the points, calculate a histogram of distances and plot it
    % not using histogram function for easier conversion to other language
    bin_bounds = linspace(0, max_dist, N_bins+1);
    bin_counts = zeros(1, N_bins);
    pt = pts(:, 1);
    for i=2:N_pts
        dist = sqrt((pt - pts(:, i)).' * (pt - pts(:, i)));
        for bin_i=1:N_bins
            if (dist >= bin_bounds(bin_i) && dist < bin_bounds(bin_i+1))
                bin_counts(bin_i) = bin_counts(bin_i) + 1;
                continue;
            end
        end
    end
    subplot(N_tries, 2, (k-1)*2 + 2);
    bar(bin_counts)
end

