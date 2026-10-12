function [paths_out, weights_out, extra] = solve_rmpr_once_from_cost( ...
    t_idx, dist_matrix, W_cost, result3, Sources, target_sat, L, M_bits, alpha_val, T_max_val, n_redundancy)

    paths_out = {};
    weights_out = [];
    extra = struct();

    in = rmpr_build_waterfill_inputs_multi_source( ...
        t_idx, dist_matrix, W_cost, result3, Sources, ...
        target_sat, L, M_bits, alpha_val, T_max_val, false);

    [w_vec, z_star] = solve_routing_optimization(in.q_sec, in.gamma_sec, n_redundancy);

    paths_out = in.paths;
    weights_out = w_vec;

    extra.q_sec = in.q_sec;
    extra.gamma_sec = in.gamma_sec;
    extra.z_star = z_star;
end
