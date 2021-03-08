Automated Invariant Checking for Airspace Manager
=================================================

As discussed in our paper, we also formally defined the Airspace Manager in 
In our paper, we decompose the proof of the system safety property into several lemmas and provide manual proof.
One lemma can be formulated as the following invariant of the Airspace Manager
and proven automatically by [Dione verfier](https://github.com/cyphyhouse/dione) with
[Dafny](https://github.com/dafny-lang/dafny).

```text
 ⋀  ⋀  AM.contr_arr[i] ∧ AM.contr_arr[j] = ∅
 i  j≠i
```

The automaton with the invariant can be found in [`UTMAirspaceManager.ioa`](UTMAirspaceManager.ioa).

Assuming Dione verifier is downloaded and installed, we use the following command to prove the invariant.
```shell
python3 -m dione ioa/UTMAirspaceManager.dione
```
