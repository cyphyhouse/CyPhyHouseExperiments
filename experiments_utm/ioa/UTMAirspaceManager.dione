StampedPoint: type = NamedTuple[t: Real, x: Real, y: Real, z: Real]
Contract: type = ISet[StampedPoint]
UID: type = Nat


@automaton
def AirspaceManager(N: nat):
    where = True

    class signature:
        @input
        def request(i: UID, contr: Contract):
            where = i < N
        @output
        def reply(i: UID, contr: Contract):
            where = i < N
        @input
        def release(i: UID, contr: Contract):
            where = i < N

    class states:
        contr_arr: Seq[Contract]
        reply_set: Set[UID]
    initially = reply_set == set() and len(contr_arr) == N and \
            forall(i, forall(j, implies(0 <= i < N and 0 <= j < N and i != j,
                                        disjoint(contr_arr[i], contr_arr[j]))))

    class transitions:
        @input
        def request(i: UID, contr: Contract):
            reply_set = reply_set + {i}
            if forall(j, implies(0 <= j < N and i != j, disjoint(contr, contr_arr[j]))):
                contr_arr[i] = contr_arr[i] + contr

        @output
        @pre(i in reply_set and contr == contr_arr[i])
        def reply(i: UID, contr: Contract):
            reply_set = reply_set - {i}

        @input
        def release(i: UID, contr: Contract):
            contr_arr[i] = contr_arr[i] - contr

    invariant_of = len(contr_arr) == N and \
        forall(i, forall(j, implies(0 <= i < N and 0 <= j < N and i != j,
                                    disjoint(contr_arr[i], contr_arr[j]))))
